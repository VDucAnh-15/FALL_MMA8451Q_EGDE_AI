/**
 * MMA8451Q FIFO Watermark Interrupt Demo with Continuous Light Sleep
 * 50Hz, ±8g, Watermark = 25 samples
 *
 * Nút 1 (D1):
 *  - đang STOP: single -> tạo file fall_n.csv và bắt đầu thu
 *  - đang RUN : single -> bỏ qua
 *  - đang RUN : double -> dừng thu
 *
 * Nút 2 (D3):
 *  - đang RUN : single -> ghi sample_id hiện tại vào file TXT
 */

#include <Arduino.h>
#include "lib_mma8451.h"
#include <string.h>
#include <SPI.h>
#include <SD.h>
#include "driver/gpio.h"
#include "esp_sleep.h"

#define LED_STATUS D0

// Seeed XIAO ESP32C3 onboard LED is active-low.
// Set to 0 if you're using an external active-high LED.
#if !defined(STATUS_LED_ACTIVE_LOW)
  #if defined(ARDUINO_SEEED_XIAO_ESP32C3)
    #define STATUS_LED_ACTIVE_LOW 1
  #else
    #define STATUS_LED_ACTIVE_LOW 0
  #endif
#endif

static inline void statusLedWrite(bool on)
{
  const uint8_t onLevel = STATUS_LED_ACTIVE_LOW ? LOW : HIGH;
  const uint8_t offLevel = STATUS_LED_ACTIVE_LOW ? HIGH : LOW;
  digitalWrite(LED_STATUS, on ? onLevel : offLevel);
}

#define MMA8451_ADDR 0x1C
#define INT1_PIN D2 

#define BUTTON_DATA_SAMPLE D1
#define BUTTON_DATA_ID_SAMPLE_FALL_TXT D3

#define SD_CS D7

#define WATERMARK_SAMPLES 25
#define DATA_RATE DR_50HZ
#define ACCEL_RANGE RANGE_8G

// Double click & debounce
#define DOUBLE_CLICK_WINDOW 400 // ms – khoảng thời gian tối đa giữa 2 lần nhấn
#define DEBOUNCE_BTN1 100       // ms – debounce cho nút 1 (tăng lên để chống rung tốt hơn)
#define DEBOUNCE_BTN2 100       // ms – debounce cho nút 2
#define BUTTON_CONFIRM_DELAY 50 // ms – thời gian xác nhận nút được nhấn

MMA8451 mma(MMA8451_ADDR);

unsigned long read_count = 0;
int file_index = 0;
bool is_recording = false;
String current_filename = "";
unsigned long sample_id = 0;

// Nút 1 – single/double
unsigned long last_btn1_press_time = 0;  // thời điểm nhấn mới nhất (đã debounce)
unsigned long last_btn1_click_time = 0;  // thời điểm click đầu tiên để chờ double
unsigned long btn1_edge_detect_time = 0; // thời điểm phát hiện cạnh xuống
bool btn1_waiting_single = false;
bool btn1_edge_detected = false; // đã phát hiện cạnh xuống chưa?
bool last_btn1_state = HIGH;

// Nút 2 – marker TXT
unsigned long last_btn2_press = 0;
bool last_btn2_state = HIGH;

// LED effect state machine
enum LedEffect { LED_OFF, LED_ON, LED_BLINK_FAST, LED_BLINK_SLOW, LED_FLASH_QUICK };
LedEffect current_led_effect = LED_OFF;
unsigned long led_effect_start_time = 0;
int led_blink_count = 0;
int led_blink_target = 0;
bool led_on = false;
bool led_effect_active = false;

String makeLabelFilenameFromCSV(const String &csvName);
void startNewFileAndRecord();
void stopRecording();
void saveMarkerToParallelTxt();
void enterContinuousSleep();
void processFIFOData();
void handleButton1();
void handleButton2();
void updateLedEffect();
void startLedEffect(LedEffect effect, int times);

void setup()
{
  // Make sure the status LED is OFF as early as possible.
  pinMode(LED_STATUS, OUTPUT);
  // If the pin was held from a previous sleep mode, release it.
  gpio_hold_dis((gpio_num_t)LED_STATUS);
  statusLedWrite(false);

  delay(500);

  if (!SD.begin(SD_CS))
  {
    while (1)
    {
    }
  }

  pinMode(BUTTON_DATA_SAMPLE, INPUT_PULLUP);
  pinMode(BUTTON_DATA_ID_SAMPLE_FALL_TXT, INPUT_PULLUP);

  if (!mma.begin())
  {
    while (1)
      delay(1000);
  }

  pinMode(INT1_PIN, INPUT);
  mma.setupFIFO_Watermark(DATA_RATE, ACCEL_RANGE, WATERMARK_SAMPLES);

  delay(1000);
}

void loop()
{
  unsigned long now = millis();

  // 0) Cập nhật LED effect (non-blocking) - chỉ khi đang có effect
  if (led_effect_active)
  {
    updateLedEffect();
  }

  // 1) Xử lý nút 1 (single/double) và nút 2 (marker) - chỉ khi LED không blink
  if (!led_effect_active)
  {
    handleButton1();
    handleButton2();
  }

  // 2) Nếu đang recording thì xử lý FIFO khi có interrupt
  if (is_recording)
  {
    if (digitalRead(INT1_PIN) == LOW)
    {
      processFIFOData();
    }

    // Cho phép nhấn nút trong lúc đang thu - chỉ khi LED không blink
    if (!led_effect_active)
    {
      handleButton1();
      handleButton2();
    }
  }

  // 3) Nếu đang chờ double click hoặc LED effect đang chạy thì KHÔNG sleep
  bool waiting_double = btn1_waiting_single &&
                        ((now - last_btn1_click_time) <= DOUBLE_CLICK_WINDOW);
  
  bool led_effect_running = (current_led_effect != LED_OFF && current_led_effect != LED_ON);

  if (!waiting_double && !led_effect_running)
  {
    enterContinuousSleep();
  }
}

/**
 * Nút 1 – phát hiện single / double click với xác nhận trạng thái
 *
 * - Khi STOP:
 *     + single: startNewFileAndRecord()
 *     + double: không làm gì
 * - Khi RUN:
 *     + single: bỏ qua
 *     + double: stopRecording()
 */
void handleButton1()
{
  unsigned long now = millis();
  bool cur = digitalRead(BUTTON_DATA_SAMPLE);

  // Bước 1: Phát hiện cạnh xuống HIGH -> LOW
  if (last_btn1_state == HIGH && cur == LOW && !btn1_edge_detected)
  {
    // Đọc lại để xác nhận (tránh glitch sau wake up)
    delayMicroseconds(50);
    cur = digitalRead(BUTTON_DATA_SAMPLE);
    
    if (cur == LOW) // Xác nhận nút thực sự đang LOW
    {
      // Ghi nhận thời điểm phát hiện cạnh
      btn1_edge_detect_time = now;
      btn1_edge_detected = true;
    }
  }

  // Bước 2: Xác nhận nút vẫn đang được giữ LOW sau khoảng delay nhỏ
  if (btn1_edge_detected && (now - btn1_edge_detect_time >= BUTTON_CONFIRM_DELAY))
  {
    // Đọc lại trạng thái nút để xác nhận
    cur = digitalRead(BUTTON_DATA_SAMPLE);

    if (cur == LOW) // Nút vẫn đang được nhấn
    {
      // Kiểm tra debounce
      if (now - last_btn1_press_time > DEBOUNCE_BTN1)
      {
        last_btn1_press_time = now;

        if (!is_recording)
        {
          // ĐANG STOP: nhấn 1 lần -> bắt đầu thu data ngay lập tức
          startNewFileAndRecord();

          // Reset mọi state liên quan double click
          btn1_waiting_single = false;
          last_btn1_click_time = 0;
        }
        else
        {
          // ĐANG RUN: kiểm tra single/double click
          if (!btn1_waiting_single)
          {
            // Click đầu tiên - bắt đầu đợi double click
            btn1_waiting_single = true;
            last_btn1_click_time = now;
          }
          else
          {
            // Click thứ 2 - kiểm tra có trong cửa sổ double click không
            if (now - last_btn1_click_time <= DOUBLE_CLICK_WINDOW)
            {
              // Double click được phát hiện -> dừng thu data
              stopRecording();
            }
            // Reset state
            btn1_waiting_single = false;
            last_btn1_click_time = 0;
          }
        }
      }
    }

    // Reset cờ phát hiện cạnh
    btn1_edge_detected = false;
  }

  // Bước 3: Reset cờ nếu nút đã được thả
  if (cur == HIGH)
  {
    btn1_edge_detected = false;
  }

  last_btn1_state = cur;

  // Timeout: nếu hết cửa sổ double click mà không có click thứ 2
  // thì reset state (bỏ qua single click khi đang recording)
  if (btn1_waiting_single &&
      (now - last_btn1_click_time > DOUBLE_CLICK_WINDOW))
  {
    btn1_waiting_single = false;
    last_btn1_click_time = 0;
  }
}

/**
 * Nút 2 – ghi marker (sample_id) vào file TXT song song
 * - Chỉ có tác dụng khi đang recording
 */
void handleButton2()
{
  unsigned long now = millis();
  bool btn2 = digitalRead(BUTTON_DATA_ID_SAMPLE_FALL_TXT);

  // Cạnh HIGH -> LOW
  if (last_btn2_state == HIGH && btn2 == LOW)
  {
    // Đọc lại để xác nhận (tránh glitch sau wake up)
    delayMicroseconds(50);
    btn2 = digitalRead(BUTTON_DATA_ID_SAMPLE_FALL_TXT);
    
    if (btn2 == LOW && (now - last_btn2_press > DEBOUNCE_BTN2))
    {
      last_btn2_press = now;
      saveMarkerToParallelTxt();
    }
  }
  last_btn2_state = btn2;
}

/**
 * Đọc FIFO theo watermark và ghi vào CSV
 */
void processFIFOData()
{
  uint8_t count = mma.getFIFOCount();

  if (count >= WATERMARK_SAMPLES)
  {
    read_count++;

    File file = SD.open(current_filename.c_str(), FILE_APPEND);
    if (file)
    {
      uint8_t samples_to_read = (count > WATERMARK_SAMPLES) ? WATERMARK_SAMPLES : count;

      for (uint8_t i = 0; i < samples_to_read; i++)
      {
        int16_t x, y, z;
        mma.readFIFOSampleRaw(x, y, z);

        sample_id++;

        file.print(x);
        file.print(",");
        file.print(y);
        file.print(",");
        file.println(z);
      }
      file.close();
    }

    mma.clearFIFOInterrupt();
  }
}

/**
 * Light sleep liên tục:
 * - Khi STOP: cho 2 nút đánh thức
 * - Khi RUN : INT1 + 2 nút đều có thể đánh thức
 */
void enterContinuousSleep()
{
  // Keep LED stable during sleep/wake (prevents brief glitches on some boards).
  statusLedWrite(false);
  digitalWrite(LED_STATUS, HIGH);
  gpio_hold_en((gpio_num_t)LED_STATUS);

  // Disable tất cả GPIO wakeup trước để tránh enable chồng
  gpio_wakeup_disable((gpio_num_t)INT1_PIN);
  gpio_wakeup_disable((gpio_num_t)BUTTON_DATA_SAMPLE);
  gpio_wakeup_disable((gpio_num_t)BUTTON_DATA_ID_SAMPLE_FALL_TXT);

  if (!is_recording)
  {
    gpio_wakeup_enable((gpio_num_t)BUTTON_DATA_SAMPLE, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)BUTTON_DATA_ID_SAMPLE_FALL_TXT, GPIO_INTR_LOW_LEVEL);
  }
  else
  {
    gpio_wakeup_enable((gpio_num_t)INT1_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)BUTTON_DATA_SAMPLE, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)BUTTON_DATA_ID_SAMPLE_FALL_TXT, GPIO_INTR_LOW_LEVEL);
  }

  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();

  // Release hold so LED effects can work again after wake.
  gpio_hold_dis((gpio_num_t)LED_STATUS);
  
  // Delay nhỏ sau khi wake up để GPIO ổn định
  delayMicroseconds(100);
}

/**
 * Từ tên CSV tạo tên TXT song song
 *  ví dụ: /fall_0.csv -> /fall_0.txt
 */
String makeLabelFilenameFromCSV(const String &csvName)
{
  String txt = csvName;
  if (txt.endsWith(".csv"))
  {
    txt.remove(txt.length() - 4);
    txt += ".txt";
  }
  else
  {
    txt += ".txt";
  }
  return txt;
}

/**
 * Bắt đầu file mới và bật trạng thái recording
 */
void startNewFileAndRecord()
{
  current_filename = "/fall_" + String(file_index) + ".csv";
  file_index++;

  File file = SD.open(current_filename.c_str(), FILE_WRITE);
  if (!file)
  {
    return;
  }
  file.println("X_ms2,Y_ms2,Z_ms2");
  file.close();

  read_count = 0;
  sample_id = 0;
  is_recording = true;

  // Hiệu ứng LED: nhấp nháy nhanh 3 lần rồi tắt
  startLedEffect(LED_BLINK_FAST, 3);
  delay(50); // Delay nhỏ sau khi LED sáng
}

/**
 * Dừng recording
 */
void stopRecording()
{
  is_recording = false;
  mma.clearFIFOInterrupt();

  // Hiệu ứng LED: nhấp nháy nhanh 2 lần rồi tắt
  startLedEffect(LED_BLINK_FAST, 2);
  delay(50); // Delay nhỏ sau khi LED sáng
}

/**
 * Ghi marker sample_id hiện tại vào file TXT song song với file CSV
 */
void saveMarkerToParallelTxt()
{
  if (!is_recording)
  {
    return;
  }

  String txtName = makeLabelFilenameFromCSV(current_filename);

  File f = SD.open(txtName.c_str(), FILE_APPEND);
  if (!f)
  {
    return;
  }

  f.println(sample_id);
  f.close();

  // Hiệu ứng LED: nhấp nháy nhanh 2 lần rồi tắt
  startLedEffect(LED_BLINK_FAST, 2);
  delay(50); // Delay nhỏ sau khi LED sáng
}

/**
 * Bắt đầu hiệu ứng LED (non-blocking)
 */
void startLedEffect(LedEffect effect, int times)
{
  current_led_effect = effect;
  led_effect_start_time = millis();
  led_blink_count = 0;
  led_blink_target = times;
  led_on = false;
  statusLedWrite(false);
  led_effect_active = true;
}

/**
 * Cập nhật LED effect - gọi trong loop() (non-blocking)
 */
void updateLedEffect()
{
  unsigned long now = millis();
  unsigned long elapsed = now - led_effect_start_time;

  switch (current_led_effect)
  {
    case LED_OFF:
      statusLedWrite(false);
      led_effect_active = false;
      break;

    // case LED_ON:
    //   digitalWrite(LED_STATUS, HIGH);
    //   break;

    case LED_BLINK_FAST: // 150ms on/off
    {
      unsigned long period = 300; // 150ms on + 150ms off
      unsigned long phase = elapsed % period;
      
      if (led_blink_count < led_blink_target)
      {
        bool new_on = (phase < 150);
        if (new_on != led_on)
        {
          led_on = new_on;
          statusLedWrite(led_on);
          if (!led_on) led_blink_count++; // Đếm khi chuyển từ ON -> OFF
        }
      }
      else
      {
        // Hoàn thành blink -> tắt hẳn
        current_led_effect = LED_OFF;
        statusLedWrite(false);
        led_effect_active = false;
      }
      break;
    }
  }
}

