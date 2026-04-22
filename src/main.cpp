/**
 * MMA8451 FIFO + Edge Impulse inference with continuous light sleep.
 *
 * Kept from original firmware:
 * - MMA8451 data rate: 50 Hz
 * - MMA8451 range: +-8g
 * - FIFO watermark: 25 samples
 * - GPIO light sleep wake via INT1
 *
 * Removed from original firmware:
 * - Button handling
 * - SD card logging
 */

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <Fall_Detech_inferencing.h>
#include "lib_mma8451.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#if !defined(LED_STATUS)
#define LED_STATUS D0
#endif

// Seeed XIAO ESP32C3 onboard LED is active-low.
#if !defined(STATUS_LED_ACTIVE_LOW)
    #if defined(ARDUINO_XIAO_ESP32C3) || defined(ARDUINO_SEEED_XIAO_ESP32C3)
    #define STATUS_LED_ACTIVE_LOW 1
  #else
    #define STATUS_LED_ACTIVE_LOW 0
  #endif
#endif

#define FALL_LED_THRESHOLD 0.80f
#define ENABLE_RUNTIME_LOG 0

static inline void statusLedWrite(bool on)
{
    const uint8_t onLevel = STATUS_LED_ACTIVE_LOW ? LOW : HIGH;
    const uint8_t offLevel = STATUS_LED_ACTIVE_LOW ? HIGH : LOW;
    digitalWrite(LED_STATUS, on ? onLevel : offLevel);
}

#define MMA8451_ADDR 0x1C
#define INT1_PIN D2

#define WATERMARK_SAMPLES 25
#define DATA_RATE DR_50HZ
#define ACCEL_RANGE RANGE_8G
#define INFERENCE_BUFFER_VALUES EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE

static MMA8451 mma(MMA8451_ADDR);
static const bool debug_nn = false;

static float inference_buffer[INFERENCE_BUFFER_VALUES];
static size_t inference_buffer_ix = 0;
static bool inference_buffer_full = false;

void ei_printf(const char *format, ...)
{
#if ENABLE_RUNTIME_LOG
    char print_buf[256];

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.print(print_buf);
    }
#else
    (void)format;
#endif
}

static void pushSampleToInferenceBuffer(int16_t x_raw, int16_t y_raw, int16_t z_raw)
{
    if (inference_buffer_full) {
        return;
    }

    if (inference_buffer_ix + 3 <= INFERENCE_BUFFER_VALUES) {
        inference_buffer[inference_buffer_ix++] = (float)x_raw;
        inference_buffer[inference_buffer_ix++] = (float)y_raw;
        inference_buffer[inference_buffer_ix++] = (float)z_raw;
    }

    if (inference_buffer_ix >= INFERENCE_BUFFER_VALUES) {
        inference_buffer_full = true;
    }
}

static void processFIFOData()
{
    uint8_t count = mma.getFIFOCount();

    if (count < WATERMARK_SAMPLES) {
        return;
    }

    // Keep wake processing cost deterministic by consuming a fixed chunk.
    uint8_t samples_to_read = WATERMARK_SAMPLES;

    for (uint8_t i = 0; i < samples_to_read; i++) {
        int16_t x_raw = 0;
        int16_t y_raw = 0;
        int16_t z_raw = 0;

        mma.readFIFOSampleRaw(x_raw, y_raw, z_raw);
        pushSampleToInferenceBuffer(x_raw, y_raw, z_raw);
    }

    mma.clearFIFOInterrupt();
}

static void enterContinuousSleep()
{
    // Wake only when MMA8451 INT1 goes low (FIFO watermark event).
    gpio_wakeup_disable((gpio_num_t)INT1_PIN);
    gpio_wakeup_enable((gpio_num_t)INT1_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    // Hold LED output level during sleep to avoid brief glitches.
    gpio_hold_en((gpio_num_t)LED_STATUS);
    esp_light_sleep_start();
    gpio_hold_dis((gpio_num_t)LED_STATUS);

    delayMicroseconds(80);
}

static void runInference(uint8_t fifo_snapshot)
{
//     ei_printf("Status: buffer=%u/%u, fifo=%u -> run inference\r\n",
//               (unsigned)inference_buffer_ix,
//               (unsigned)INFERENCE_BUFFER_VALUES,
//               (unsigned)fifo_snapshot);

    signal_t signal;
    int err = numpy::signal_from_buffer(inference_buffer, INFERENCE_BUFFER_VALUES, &signal);
    if (err != 0) {
        // ei_printf("ERR: signal_from_buffer failed (%d)\r\n", err);
        inference_buffer_ix = 0;
        inference_buffer_full = false;
        return;
    }

    ei_impulse_result_t result = {0};
    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        // ei_printf("ERR: run_classifier failed (%d)\r\n", err);
        inference_buffer_ix = 0;
        inference_buffer_full = false;
        return;
    }

    // ei_printf("Predictions (DSP: %d ms, Classification: %d ms, Anomaly: %d ms)\r\n",
    //           result.timing.dsp,
    //           result.timing.classification,
    //           result.timing.anomaly);

    float fall_score = 0.0f;
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        // ei_printf("  %s: %.5f\r\n", result.classification[ix].label, result.classification[ix].value);
        if (strcmp(result.classification[ix].label, "FALL") == 0) {
            fall_score = result.classification[ix].value;
        }
    }

    const bool fall_detected = (fall_score >= FALL_LED_THRESHOLD);
    statusLedWrite(fall_detected);
    // ei_printf("  FALL score: %.5f (threshold: %.2f) -> LED: %s\r\n",
    //           fall_score,
    //           FALL_LED_THRESHOLD,
    //           fall_detected ? "ON" : "OFF");

    

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    // ei_printf("  anomaly score: %.3f\r\n", result.anomaly);
#endif

    inference_buffer_ix = 0;
    inference_buffer_full = false;
}

void setup()
{
    pinMode(LED_STATUS, OUTPUT);
    gpio_hold_dis((gpio_num_t)LED_STATUS);
    statusLedWrite(false);

#if ENABLE_RUNTIME_LOG
    Serial.begin(115200);
    uint32_t serial_wait_start = millis();
    while (!Serial && (millis() - serial_wait_start < 3000)) {
        delay(10);
    }
#endif

    // ei_printf("Edge Impulse MMA8451 inferencing\r\n");
    // ei_printf("Required model axes: %s\r\n", EI_CLASSIFIER_FUSION_AXES_STRING);

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        // ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME must be 3, got %d\r\n",
        //           EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);
        while (1) {
            // ei_printf("ERR: blocked by axis-count mismatch\r\n");
            delay(1000);
        }
    }

    if (!mma.begin()) {
        // ei_printf("ERR: MMA8451 init failed\r\n");
        while (1) {
            // ei_printf("ERR: blocked by MMA8451 init failure\r\n");
            delay(1000);
        }
    }

    pinMode(INT1_PIN, INPUT);
    mma.setupFIFO_Watermark(DATA_RATE, ACCEL_RANGE, WATERMARK_SAMPLES);
    mma.clearFIFOInterrupt();

    // ei_printf("MMA8451 config kept: DR_50HZ, RANGE_8G, WATERMARK_25\r\n");
    // ei_printf("Using RAW accel values for inference\r\n");
    // ei_printf("Model frame size: %d values\r\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    // ei_printf("Run inference each time buffer reaches %d values\r\n", INFERENCE_BUFFER_VALUES);
    // ei_printf("Light sleep: ON (wake by INT1 FIFO watermark)\r\n");
}

void loop()
{
    if (!inference_buffer_full) {
        enterContinuousSleep();
        processFIFOData();
    }

    if (inference_buffer_full) {
        uint8_t fifo_snapshot = mma.getFIFOCount();
        runInference(fifo_snapshot);
    }
}

#if !defined(EI_CLASSIFIER_SENSOR) || (EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_FUSION && EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER)
#error "Invalid model for current sensor"
#endif
