/*!
 *  @file Adafruit_BNO08x.cpp
 *
 *  @mainpage Adafruit BNO08x 9-DOF Orientation IMU Fusion Breakout
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Library for the BNO08x 9-DOF Orientation IMU Fusion
 * Breakout
 *
 * 	This is a library for the Adafruit BNO08x breakout:
 * 	https://www.adafruit.com/product/4754
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  This library depends on the Adafruit Unified Sensor library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#define I2C_MAX_BUFFER_SIZE 256

#include "Adafruit_BNO08x.h"

#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <functional>
#include <memory>

#include "pico.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

static Adafruit_BNO08x *self;
static sh2_SensorValue_t *_sensor_value = nullptr;
static bool _reset_occurred = false;

static int i2chal_write(/* Adafruit_BNO08x *self */ sh2_Hal_t *hal,
                        uint8_t *pBuffer, unsigned len);
static int i2chal_read(/* Adafruit_BNO08x *self */ sh2_Hal_t *hal,
                       uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static void i2chal_close(sh2_Hal_t *self);
static int i2chal_open(/* Adafruit_BNO08x *self */ sh2_Hal_t *hal);

static uint32_t hal_getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);
static void hal_hardwareReset(Adafruit_BNO08x *self);

/**
 * @brief Construct a new Adafruit_BNO08x::Adafruit_BNO08x object
 *
 */

/**
 * @brief Construct a new Adafruit_BNO08x::Adafruit_BNO08x object
 *
 * @param reset_pin The arduino pin # connected to the BNO Reset pin
 */
Adafruit_BNO08x::Adafruit_BNO08x(int8_t reset_pin) : _reset_pin(reset_pin) {}

/**
 * @brief Destroy the Adafruit_BNO08x::Adafruit_BNO08x object
 *
 */
Adafruit_BNO08x::~Adafruit_BNO08x(void) {
  // if (temp_sensor)
  //   delete temp_sensor;
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            The unique ID to differentiate the sensors from others
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_BNO08x::begin_I2C(uint8_t i2c_address, i2c_inst_t *wire, uint sda,
                                uint scl, int32_t sensor_id) {
  _i2c_dev = wire;
  _i2c_addr = i2c_address;

  if (self != nullptr) {
    printf("BNO08x - Only one instance of BNO08x is supported at a time.");
    return false;
  }

  self = this;

  // if (i2c_dev) {
  //   delete i2c_dev; // remove old interface
  // }

  // set up the reset pin
  gpio_init(_reset_pin);
  gpio_set_dir(_reset_pin, GPIO_OUT);

  // initialize i2c
  i2c_init(_i2c_dev, 100 * 1000);  // TODO: figure out speed
  gpio_set_function(sda, GPIO_FUNC_I2C);
  gpio_set_function(scl, GPIO_FUNC_I2C);
  gpio_pull_up(sda);
  gpio_pull_up(scl);

  bi_decl(bi_2pins_with_func(sda, scl, GPIO_FUNC_I2C));

  // check i2c
  uint8_t rxdata;
  int ret = i2c_read_blocking(_i2c_dev, i2c_address, &rxdata, 1, false);
  if (ret < 0) {
    printf("Failed to connect to i2c.");
    return false;
  }

  // dunno how to get bind to work
  // _HAL.open = std::bind(&i2chal_open, this, std::placeholders::_1);
  // _HAL.close = i2chal_close;
  // _HAL.read = std::bind(&i2chal_read, this, std::placeholders::_1,
  //                       std::placeholders::_2, std::placeholders::_3);
  // _HAL.write = std::bind(&i2chal_write, this, std::placeholders::_1,
  //                        std::placeholders::_2, std::placeholders::_3);

  _HAL.open = i2chal_open;
  _HAL.close = i2chal_close;
  _HAL.read = i2chal_read;
  _HAL.write = i2chal_write;

  _HAL.getTimeUs = hal_getTimeUs;

  return _init(sensor_id);
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_BNO08x::_init(int32_t sensor_id) {
  int status;

  hardwareReset();

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&_HAL, hal_callback, NULL);
  if (status != SH2_OK) {
    return false;
  }

  // Check connection partially by getting the product id's
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK) {
    return false;
  }

  // Register sensor listener
  sh2_setSensorCallback(sensorHandler, NULL);

  return true;
}

/**
 * @brief Reset the device using the Reset pin
 *
 */
void Adafruit_BNO08x::hardwareReset(void) { hal_hardwareReset(this); }

/**
 * @brief Check if a reset has occured
 *
 * @return true: a reset has occured false: no reset has occoured
 */
bool Adafruit_BNO08x::wasReset(void) {
  bool x = _reset_occurred;
  _reset_occurred = false;

  return x;
}

/**
 * @brief Fill the given sensor value object with a new report
 *
 * @param value Pointer to an sh2_SensorValue_t struct to fil
 * @return true: The report object was filled with a new report
 * @return false: No new report available to fill
 */
bool Adafruit_BNO08x::getSensorEvent(sh2_SensorValue_t *value) {
  _sensor_value = value;

  value->timestamp = 0;

  sh2_service();

  if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @return true: success false: failure
 */
bool Adafruit_BNO08x::enableReport(sh2_SensorId_t sensorId,
                                   uint32_t interval_us) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  config.reportInterval_us = interval_us;
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

/**************************************** I2C interface
 * ***********************************************************/

static int i2chal_open(/* Adafruit_BNO08x *self */ sh2_Hal_t *hal) {
  uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  bool success = false;
  for (uint8_t attempts = 0; attempts < 5; attempts++) {
    int ret = i2c_write_blocking(self->_i2c_dev, self->_i2c_addr, softreset_pkt,
                                 5, false);
    if (ret == 5) {
      success = true;
      break;
    }

    sleep_ms(30);
  }
  if (!success) return -1;
  sleep_ms(300);
  return 0;
}

static void i2chal_close(sh2_Hal_t *self) {
  // Serial.println("I2C HAL close");
}

static int i2chal_read(/* Adafruit_BNO08x *self */ sh2_Hal_t *hal,
                       uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
  uint8_t header[4];

  int ret =
      i2c_read_blocking(self->_i2c_dev, self->_i2c_addr, header, 4, false);
  if (ret != 4) {
    printf("Failed to read from i2c!");
    return 0;
  }

  // Determine amount to read
  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  /*
  Serial.print("Read SHTP header. ");
  Serial.print("Packet size: ");
  Serial.print(packet_size);
  Serial.print(" & buffer size: ");
  Serial.println(len);
  */

  size_t i2c_buffer_max =
      I2C_MAX_BUFFER_SIZE;  // from maxBufferSize(), 256 is arbitrary.

  if (packet_size > len) {
    // packet wouldn't fit in our buffer
    return 0;
  }
  // the number of non-header bytes to read
  uint16_t cargo_remaining = packet_size;
  uint8_t i2c_buffer[i2c_buffer_max];
  uint16_t read_size;
  uint16_t cargo_read_amount = 0;
  bool first_read = true;

  while (cargo_remaining > 0) {
    if (first_read) {
      read_size = std::min(i2c_buffer_max, (size_t)cargo_remaining);
    } else {
      read_size = std::min(i2c_buffer_max, (size_t)cargo_remaining + 4);
    }

    // Serial.print("Reading from I2C: "); Serial.println(read_size);
    // Serial.print("Remaining to read: "); Serial.println(cargo_remaining);

    int ret = i2c_read_blocking(self->_i2c_dev, self->_i2c_addr, i2c_buffer,
                                read_size, false);
    if (ret != read_size) {
      printf("Failed to read from i2c!");
      return 0;
    }

    if (first_read) {
      // The first time we're saving the "original" header, so include it in the
      // cargo count
      cargo_read_amount = read_size;
      memcpy(pBuffer, i2c_buffer, cargo_read_amount);
      first_read = false;
    } else {
      // this is not the first read, so copy from 4 bytes after the beginning of
      // the i2c buffer to skip the header included with every new i2c read and
      // don't include the header in the amount of cargo read
      cargo_read_amount = read_size - 4;
      memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
    }
    // advance our pointer by the amount of cargo read
    pBuffer += cargo_read_amount;
    // mark the cargo as received
    cargo_remaining -= cargo_read_amount;
  }

  /*
  for (int i=0; i<packet_size; i++) {
    Serial.print(pBufferOrig[i], HEX);
    Serial.print(", ");
    if (i % 16 == 15) Serial.println();
  }
  Serial.println();
  */

  return packet_size;
}

static int i2chal_write(/* Adafruit_BNO08x *self */ sh2_Hal_t *hal,
                        uint8_t *pBuffer, unsigned len) {
  size_t i2c_buffer_max = I2C_MAX_BUFFER_SIZE;  // from maxBufferSize(), 256 is
                                                // arbitrary.

  /*
  Serial.print("I2C HAL write packet size: ");
  Serial.print(len);
  Serial.print(" & max buffer size: ");
  Serial.println(i2c_buffer_max);
  */

  uint16_t write_size = std::min(i2c_buffer_max, len);
  int ret = i2c_write_blocking(self->_i2c_dev, self->_i2c_addr, pBuffer,
                               write_size, false);

  if (ret != write_size) {
    printf("Failed to write to i2c!");
    return 0;
  }

  return write_size;
}

/**************************************** HAL interface
 * ***********************************************************/

static void hal_hardwareReset(Adafruit_BNO08x *self) {
  if (self->_reset_pin != -1) {
    // Serial.println("BNO08x Hardware reset");
    gpio_put(self->_reset_pin, 1);
    sleep_ms(10);
    gpio_put(self->_reset_pin, 0);
    sleep_ms(10);
    gpio_put(self->_reset_pin, 1);
    sleep_ms(10);
  }
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
  return to_us_since_boot(get_absolute_time());
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    // Serial.println("Reset!");
    _reset_occurred = true;
  }
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

  // Serial.println("Got an event!");

  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    printf("BNO08x - Error decoding sensor event");
    _sensor_value->timestamp = 0;
    return;
  }
}
