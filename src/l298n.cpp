#include "l298n.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <cstdio>

L298N::L298N(uint enableFwd, uint enableRev, uint pwm)
    : enableFwd(enableFwd), enableRev(enableRev), pwm(pwm) {
  // set up the two IN pins.
  gpio_init(enableFwd);
  gpio_set_dir(enableFwd, GPIO_OUT);

  gpio_init(enableRev);
  gpio_set_dir(enableRev, GPIO_OUT);

  // set up the PWM for Enable
  pwmChannel = pwm_gpio_to_channel(pwm);
  pwmSlice = pwm_gpio_to_slice_num(pwm);
  printf("channel: %d\nslice: %d\n", pwmChannel, pwmSlice);

  pwm_config cfg = pwm_get_default_config();
  gpio_set_function(pwm, GPIO_FUNC_PWM);
  pwm_config_set_clkdiv(&cfg, 6.f);
  pwm_init(pwmSlice, &cfg, true);

  // Start with 0% duty cycle
  pwm_set_enabled(pwmSlice, true);
  pwm_set_chan_level(pwmSlice, pwmChannel, 0);
}

void L298N::spin(bool forwards, float speed) {
  spin(forwards ? Forwards : Backwards, speed);
}

void L298N::spin(Direction direction, float speed) {
  uint8_t fwd = direction == Forwards; // 1 or 0

  gpio_put(enableFwd, fwd);
  gpio_put(enableRev, 1 - fwd);

  // float cycle = (speed * 655.0f) / 100.f;
  uint16_t cycle = (uint16_t)((speed * 65535.0f) / 100.0f);

  // printf("%d: %d, %d: %d, cycle (s: %d, c: %d): %hu\n", enableFwd, fwd,
  //        enableRev, 1 - fwd, pwmSlice, pwmChannel, cycle);

  // set pwm
  pwm_set_enabled(pwmSlice, true);
  pwm_set_chan_level(pwmSlice, pwmChannel, cycle);
}

void L298N::stop() {
  gpio_put(enableFwd, 0);
  gpio_put(enableRev, 0);
}
