#include <stdio.h>
#include "boards/pico.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "quadrature_encoder.pio.h"
#include "mpu9250/mpu9250.h"
#include "hardware/i2c.h"

const uint START_BUTTON_PIN = 15;
const uint BEEPER_PIN = 14;
const uint LEFT_WHEEL_ENCODER = 10; // AB on 10,11
const uint RIGHT_WHEEL_ENCODER = 12; // AB on 12,13

int main() {
  // picotool configuration
  bi_decl(bi_program_description("Science Olympiad Robot Tour - derock@derock.dev"));

  stdio_init_all();

  // initialize GPIO
  gpio_init(BEEPER_PIN);
  gpio_set_dir(BEEPER_PIN, GPIO_OUT);

  // initialize PIOs
  pio_add_program(pio0, &quadrature_encoder_program);
  quadrature_encoder_program_init(pio0, 0, LEFT_WHEEL_ENCODER, 0);

  pio_add_program(pio1, &quadrature_encoder_program);
  quadrature_encoder_program_init(pio1, 0, RIGHT_WHEEL_ENCODER, 0);


  // beep once  
  gpio_put(BEEPER_PIN, 1);
  sleep_ms(50);
  gpio_put(BEEPER_PIN, 0);


  // setup MPU
  i2c_init(i2c_default, 100 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  printf("i2c init success");
  MPU9250* mpu = new MPU9250();
  mpu->setup(0x68);
  printf("mpu init, calibrating...");
  mpu->calibrateMag();
  printf("mpu ok!");

  // beep once  
  gpio_put(BEEPER_PIN, 1);
  sleep_ms(200);
  gpio_put(BEEPER_PIN, 0);

  // main loop
  while (true) {
    int left = quadrature_encoder_get_count(pio0, 0);
    int right = quadrature_encoder_get_count(pio1, 0);
    int heading = mpu->getMagY();

    printf("left,right: %d, %d, %d\n", left, right, heading);
    sleep_ms(250);
  }
}
