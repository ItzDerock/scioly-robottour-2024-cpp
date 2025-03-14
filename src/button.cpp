#include "hardware/gpio.h"
#include "imu.h"
#include "pico/types.h"

#define DEBOUNCE_DELAY_MS 75       // Debounce delay in milliseconds
#define MULTI_CLICK_TIMEOUT_MS 500 // Timeout to detect multiple clicks

/**
 * @brief Waits for button clicks with debounce and multi-click support, while
 * blinking an LED based on calibration status.
 *
 * This function performs several tasks:
 * - Waits for button presses on the specified button pin
 * - Debounces the button input to avoid false triggers
 * - Counts multiple clicks within a timeout window
 * - Blinks an LED at different intervals based on the calibration status:
 *   - Un_calibrated: 1 second interval
 *   - Low accuracy: 500ms interval
 *   - Medium accuracy: 100ms interval
 *   - High accuracy: LED stays on
 * - Returns the number of clicks detected within the timeout window
 * - Avoids long sleep operations to ensure button detection remains responsive
 *
 * @param button_pin GPIO pin number of the button to monitor
 * @param led_pin GPIO pin number of the LED to control
 * @return int Number of button clicks detected within the timeout window
 */
int waitForMainButton(uint button_pin, uint led_pin) {
  int clicks = 0;
  uint64_t lastPressTime = 0;
  bool ledState = false;
  uint64_t lastLedToggle = 0;
  uint ledBlinkInterval = 0;

  // Get initial calibration status
  CalibrationStatus status = getCalibrationStatus();
  ledBlinkInterval = (status == CALIBRATION_STATUS_HIGH_ACCURACY)  ? 0
                     : (status == CALIBRATION_STATUS_UNCALIBRATED) ? 1000
                                                                   : // 1s
                         (status == CALIBRATION_STATUS_LOW_ACCURACY) ? 500
                                                                     : // 500ms
                         100; // 100ms for medium accuracy

  while (true) {
    // Update LED state based on calibration status
    status = getCalibrationStatus();
    ledBlinkInterval = (status == CALIBRATION_STATUS_HIGH_ACCURACY)  ? 0
                       : (status == CALIBRATION_STATUS_UNCALIBRATED) ? 1000
                       : (status == CALIBRATION_STATUS_LOW_ACCURACY) ? 500
                                                                     : 100;

    // Handle LED blinking
    if (ledBlinkInterval > 0) {
      uint64_t currentTime = to_ms_since_boot(get_absolute_time());
      if (currentTime - lastLedToggle >= ledBlinkInterval) {
        ledState = !ledState;
        gpio_put(led_pin, ledState);
        lastLedToggle = currentTime;
      }
    } else {
      // For high accuracy, keep LED on
      if (!ledState) {
        ledState = true;
        gpio_put(led_pin, ledState);
      }
    }

    // Check for button press
    if (!gpio_get(button_pin)) {
      // Debounce by waiting for DEBOUNCE_DELAY_MS
      uint64_t pressTime = to_ms_since_boot(get_absolute_time());
      if (pressTime - lastPressTime > DEBOUNCE_DELAY_MS) {
        // Confirm button is still pressed
        sleep_ms(DEBOUNCE_DELAY_MS);
        if (!gpio_get(button_pin)) {
          clicks++;
          lastPressTime = pressTime;
        }
      }
    }

    // Check if we should exit based on timeout
    if (clicks > 0 && to_ms_since_boot(get_absolute_time()) - lastPressTime >
                          MULTI_CLICK_TIMEOUT_MS) {
      break; // Exit the loop and return the number of clicks
    }

    // Busy-wait with small delays to avoid blocking button detection
    sleep_ms(1);
  }

  return clicks;
}
