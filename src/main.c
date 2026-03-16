// Sensirion SCD40 CO2/Temperature/Humidity Sensor - Wokwi Custom Chip
// I2C command-based protocol implementation (NOT register-based)
// Reference: Sensirion SCD4x Datasheet

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

#define SCD40_I2C_ADDR 0x62

#define CMD_START_PERIODIC  0x21B1
#define CMD_STOP_PERIODIC   0x3F86
#define CMD_READ_MEASUREMENT 0xEC05
#define CMD_GET_SERIAL      0x3682
#define CMD_GET_DATA_READY  0xE4B8

typedef enum {
  STATE_IDLE,
  STATE_MEASURING
} scd40_state_t;

typedef struct {
  scd40_state_t state;
  uint16_t command_buffer;
  uint8_t cmd_byte_count;
  uint8_t response[9];
  uint8_t response_len;
  uint8_t response_idx;
  bool data_ready;
  bool first_measurement_done;
  uint32_t co2_attr;
  uint32_t temp_attr;
  uint32_t hum_attr;
  timer_t measurement_timer;
} chip_state_t;

static uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x31);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

static void prepare_measurement_response(chip_state_t *chip) {
  float co2_value = attr_read_float(chip->co2_attr);
  float temperature = attr_read_float(chip->temp_attr);
  float humidity = attr_read_float(chip->hum_attr);

  uint16_t co2_raw = (uint16_t)co2_value;
  uint16_t temp_raw = (uint16_t)((temperature + 45.0f) / 175.0f * 65535.0f);
  uint16_t hum_raw = (uint16_t)(humidity / 100.0f * 65535.0f);

  chip->response[0] = (uint8_t)(co2_raw >> 8);
  chip->response[1] = (uint8_t)(co2_raw & 0xFF);
  chip->response[2] = crc8(&chip->response[0], 2);

  chip->response[3] = (uint8_t)(temp_raw >> 8);
  chip->response[4] = (uint8_t)(temp_raw & 0xFF);
  chip->response[5] = crc8(&chip->response[3], 2);

  chip->response[6] = (uint8_t)(hum_raw >> 8);
  chip->response[7] = (uint8_t)(hum_raw & 0xFF);
  chip->response[8] = crc8(&chip->response[6], 2);

  chip->response_len = 9;
}

static void prepare_serial_response(chip_state_t *chip) {
  chip->response[0] = 0x00;
  chip->response[1] = 0x01;
  chip->response[2] = crc8(&chip->response[0], 2);

  chip->response[3] = 0x00;
  chip->response[4] = 0x02;
  chip->response[5] = crc8(&chip->response[3], 2);

  chip->response[6] = 0x00;
  chip->response[7] = 0x03;
  chip->response[8] = crc8(&chip->response[6], 2);

  chip->response_len = 9;
}

static void prepare_data_ready_response(chip_state_t *chip) {
  if (chip->data_ready) {
    chip->response[0] = 0x80;
    chip->response[1] = 0x00;
  } else {
    chip->response[0] = 0x00;
    chip->response[1] = 0x00;
  }
  chip->response[2] = crc8(&chip->response[0], 2);
  chip->response_len = 3;
}

static void process_command(chip_state_t *chip) {
  switch (chip->command_buffer) {
    case CMD_START_PERIODIC:
      chip->state = STATE_MEASURING;
      chip->data_ready = false;
      chip->first_measurement_done = false;
      timer_start(chip->measurement_timer, 100000, false);
      break;

    case CMD_STOP_PERIODIC:
      chip->state = STATE_IDLE;
      chip->data_ready = false;
      chip->first_measurement_done = false;
      timer_stop(chip->measurement_timer);
      break;

    case CMD_READ_MEASUREMENT:
      if (chip->state == STATE_MEASURING && chip->data_ready) {
        prepare_measurement_response(chip);
        chip->response_idx = 0;
      }
      break;

    case CMD_GET_SERIAL:
      prepare_serial_response(chip);
      chip->response_idx = 0;
      break;

    case CMD_GET_DATA_READY:
      prepare_data_ready_response(chip);
      chip->response_idx = 0;
      break;

    default:
      break;
  }
}

static void measurement_timer_callback(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  chip->data_ready = true;
  if (!chip->first_measurement_done) {
    chip->first_measurement_done = true;
    timer_start(chip->measurement_timer, 5000000, true);
  }
}

static bool on_i2c_connect(void *user_data, uint32_t address, bool connect) {
  chip_state_t *chip = (chip_state_t *)user_data;
  (void)address;
  (void)connect;
  chip->cmd_byte_count = 0;
  return true;
}

static uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (chip->response_idx < chip->response_len) {
    return chip->response[chip->response_idx++];
  }
  return 0xFF;
}

static bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (chip->cmd_byte_count == 0) {
    chip->command_buffer = (uint16_t)((uint16_t)data << 8);
    chip->cmd_byte_count = 1;
  } else if (chip->cmd_byte_count == 1) {
    chip->command_buffer |= data;
    chip->cmd_byte_count = 2;
    process_command(chip);
  }
  // Bytes beyond the 2-byte command word are ACKed and ignored
  return true;
}

static void on_i2c_disconnect(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  chip->cmd_byte_count = 0;
}

void chip_init(void) {
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));

  chip->co2_attr = attr_init_float("co2", 800.0f);
  chip->temp_attr = attr_init_float("temperature", 25.0f);
  chip->hum_attr = attr_init_float("humidity", 50.0f);

  const timer_config_t timer_cfg = {
    .user_data = chip,
    .callback = measurement_timer_callback,
  };
  chip->measurement_timer = timer_init(&timer_cfg);

  const i2c_config_t i2c_cfg = {
    .user_data = chip,
    .address = SCD40_I2C_ADDR,
    .scl = pin_init("SCL", INPUT),
    .sda = pin_init("SDA", INPUT),
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect,
  };
  i2c_init(&i2c_cfg);

  printf("SCD40 sensor initialized at 0x%02X\n", SCD40_I2C_ADDR);
}
