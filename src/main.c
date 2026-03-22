// Sensirion SCD40 CO2/Temperature/Humidity Sensor - Wokwi Custom Chip
// Production-grade I2C command protocol implementation (20 commands)
// Reference: Sensirion SCD4x Datasheet

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

#define SCD40_I2C_ADDR 0x62

// Command codes (Table 9)
#define CMD_START_PERIODIC       0x21B1
#define CMD_READ_MEASUREMENT     0xEC05
#define CMD_STOP_PERIODIC        0x3F86
#define CMD_GET_SERIAL           0x3682
#define CMD_GET_DATA_READY       0xE4B8
#define CMD_SET_TEMP_OFFSET      0x241D
#define CMD_GET_TEMP_OFFSET      0x2318
#define CMD_SET_ALTITUDE         0x2427
#define CMD_GET_ALTITUDE         0x2322
#define CMD_SET_AMBIENT_PRESSURE 0xE000
#define CMD_FORCED_RECAL         0x362F
#define CMD_SET_ASC              0x2416
#define CMD_GET_ASC              0x2313
#define CMD_START_LOW_POWER      0x21AC
#define CMD_PERSIST_SETTINGS     0x3615
#define CMD_SELF_TEST            0x3639
#define CMD_FACTORY_RESET        0x3632
#define CMD_REINIT               0x3646
#define CMD_POWER_DOWN           0x36E0
#define CMD_WAKE_UP              0x36F6
#define CMD_GET_SENSOR_VARIANT        0x202F
#define CMD_SET_ASC_TARGET            0x243A
#define CMD_GET_ASC_TARGET            0x233F
#define CMD_MEASURE_SINGLE_SHOT       0x219D
#define CMD_MEASURE_SINGLE_SHOT_RHT   0x2196
#define CMD_SET_ASC_INITIAL_PERIOD    0x2445
#define CMD_GET_ASC_INITIAL_PERIOD    0x2340
#define CMD_SET_ASC_STANDARD_PERIOD   0x244E
#define CMD_GET_ASC_STANDARD_PERIOD   0x234B

// Defaults
#define DEFAULT_TEMP_OFFSET 1498   // ~4.0C
#define DEFAULT_ALTITUDE    0
#define DEFAULT_ASC         true
#define DEFAULT_ASC_TARGET           400
#define DEFAULT_ASC_INITIAL_PERIOD   44
#define DEFAULT_ASC_STANDARD_PERIOD  156

typedef enum {
  STATE_IDLE,
  STATE_MEASURING,
  STATE_POWERED_DOWN
} scd40_state_t;

typedef struct {
  scd40_state_t state;
  uint8_t write_buf[5];
  uint8_t write_count;
  uint16_t current_cmd;
  uint8_t response[9];
  uint8_t response_len;
  uint8_t response_idx;
  bool data_ready;
  bool first_measurement_done;
  bool low_power_mode;
  uint64_t busy_until;

  // Calibration (active)
  uint16_t temperature_offset_raw;
  uint16_t sensor_altitude;
  uint16_t ambient_pressure;
  bool asc_enabled;

  // Calibration (persisted via persist_settings)
  uint16_t stored_temp_offset;
  uint16_t stored_altitude;
  bool stored_asc;
  bool has_persisted;

  // Extended calibration
  uint16_t asc_target;
  uint16_t asc_initial_period;
  uint16_t asc_standard_period;
  uint16_t stored_asc_target;
  uint16_t stored_asc_initial_period;
  uint16_t stored_asc_standard_period;
  bool has_measured;
  bool single_shot;
  bool single_shot_rht_only;
  uint32_t eeprom_write_count;

  // Hardware attrs
  uint32_t variant_attr;
  uint32_t serial0_attr;
  uint32_t serial1_attr;
  uint32_t serial2_attr;
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

static void reset_to_defaults(chip_state_t *chip) {
  chip->temperature_offset_raw = DEFAULT_TEMP_OFFSET;
  chip->sensor_altitude = DEFAULT_ALTITUDE;
  chip->ambient_pressure = 0;
  chip->asc_enabled = DEFAULT_ASC;
  chip->asc_target = DEFAULT_ASC_TARGET;
  chip->asc_initial_period = DEFAULT_ASC_INITIAL_PERIOD;
  chip->asc_standard_period = DEFAULT_ASC_STANDARD_PERIOD;
}

static void prepare_measurement_response(chip_state_t *chip) {
  float co2_value = chip->single_shot_rht_only ? 0.0f : attr_read_float(chip->co2_attr);
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
  uint16_t s0 = (uint16_t)attr_read(chip->serial0_attr);
  uint16_t s1 = (uint16_t)attr_read(chip->serial1_attr);
  uint16_t s2 = (uint16_t)attr_read(chip->serial2_attr);
  chip->response[0] = (uint8_t)(s0 >> 8); chip->response[1] = (uint8_t)(s0 & 0xFF);
  chip->response[2] = crc8(&chip->response[0], 2);
  chip->response[3] = (uint8_t)(s1 >> 8); chip->response[4] = (uint8_t)(s1 & 0xFF);
  chip->response[5] = crc8(&chip->response[3], 2);
  chip->response[6] = (uint8_t)(s2 >> 8); chip->response[7] = (uint8_t)(s2 & 0xFF);
  chip->response[8] = crc8(&chip->response[6], 2);
  chip->response_len = 9;
}

static void prepare_data_ready_response(chip_state_t *chip) {
  if (chip->data_ready) {
    chip->response[0] = 0x80;
    chip->response[1] = 0x01;
  } else {
    chip->response[0] = 0x00;
    chip->response[1] = 0x00;
  }
  chip->response[2] = crc8(&chip->response[0], 2);
  chip->response_len = 3;
}

static void prepare_word_response(chip_state_t *chip, uint16_t word) {
  chip->response[0] = (uint8_t)(word >> 8);
  chip->response[1] = (uint8_t)(word & 0xFF);
  chip->response[2] = crc8(&chip->response[0], 2);
  chip->response_len = 3;
  chip->response_idx = 0;
}

static bool command_has_payload(uint16_t cmd) {
  switch (cmd) {
    case CMD_SET_TEMP_OFFSET:
    case CMD_SET_ALTITUDE:
    case CMD_SET_AMBIENT_PRESSURE:
    case CMD_FORCED_RECAL:
    case CMD_SET_ASC:
    case CMD_SET_ASC_TARGET:
    case CMD_SET_ASC_INITIAL_PERIOD:
    case CMD_SET_ASC_STANDARD_PERIOD:
      return true;
    default:
      return false;
  }
}

static bool command_requires_scd41_or_scd43(uint16_t cmd) {
  switch (cmd) {
    case CMD_POWER_DOWN: case CMD_WAKE_UP:
    case CMD_MEASURE_SINGLE_SHOT: case CMD_MEASURE_SINGLE_SHOT_RHT:
    case CMD_SET_ASC_INITIAL_PERIOD: case CMD_GET_ASC_INITIAL_PERIOD:
    case CMD_SET_ASC_STANDARD_PERIOD: case CMD_GET_ASC_STANDARD_PERIOD:
      return true;
    default: return false;
  }
}

// State validation: returns true if command is allowed in current state
static bool command_allowed(chip_state_t *chip, uint16_t cmd) {
  if (command_requires_scd41_or_scd43(cmd) && attr_read(chip->variant_attr) == 0) {
    return false;
  }
  if (chip->state == STATE_POWERED_DOWN) {
    return cmd == CMD_WAKE_UP;
  }
  if (chip->state == STATE_MEASURING) {
    switch (cmd) {
      case CMD_READ_MEASUREMENT:
      case CMD_STOP_PERIODIC:
      case CMD_GET_DATA_READY:
      case CMD_SET_AMBIENT_PRESSURE:
        return true;
      default:
        return false;
    }
  }
  // STATE_IDLE: all commands except read_measurement and wake_up
  if (cmd == CMD_READ_MEASUREMENT) return false;
  if (cmd == CMD_WAKE_UP) return false;
  return true;
}

static void process_command(chip_state_t *chip, uint16_t cmd) {
  if (!command_allowed(chip, cmd)) return;

  chip->response_len = 0;
  chip->response_idx = 0;

  switch (cmd) {
    case CMD_START_PERIODIC:
      chip->state = STATE_MEASURING;
      chip->data_ready = false;
      chip->first_measurement_done = false;
      chip->low_power_mode = false;
      chip->single_shot = false;
      chip->single_shot_rht_only = false;
      timer_start(chip->measurement_timer, 5000000, false); // 5s first measurement
      break;

    case CMD_START_LOW_POWER:
      chip->state = STATE_MEASURING;
      chip->data_ready = false;
      chip->first_measurement_done = false;
      chip->low_power_mode = true;
      chip->single_shot = false;
      chip->single_shot_rht_only = false;
      timer_start(chip->measurement_timer, 30000000, false); // 30s first measurement
      break;

    case CMD_STOP_PERIODIC:
      chip->state = STATE_IDLE;
      chip->data_ready = false;
      chip->first_measurement_done = false;
      chip->single_shot = false;
      chip->single_shot_rht_only = false;
      timer_stop(chip->measurement_timer);
      chip->busy_until = get_sim_nanos() + 500000000ULL; // 500ms
      break;

    case CMD_READ_MEASUREMENT:
      if (chip->data_ready) {
        prepare_measurement_response(chip);
        chip->response_idx = 0;
        chip->data_ready = false; // cleared on read (datasheet 3.6.2)
        if (chip->single_shot) {
          chip->state = STATE_IDLE;
          chip->single_shot = false;
          chip->single_shot_rht_only = false;
        }
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

    case CMD_GET_TEMP_OFFSET:
      prepare_word_response(chip, chip->temperature_offset_raw);
      break;

    case CMD_GET_ALTITUDE:
      prepare_word_response(chip, chip->sensor_altitude);
      break;

    case CMD_GET_ASC:
      prepare_word_response(chip, chip->asc_enabled ? 0x0001 : 0x0000);
      break;

    case CMD_GET_SENSOR_VARIANT: {
      uint32_t v = attr_read(chip->variant_attr);
      prepare_word_response(chip, (uint16_t)(v << 12));
      break;
    }

    case CMD_GET_ASC_TARGET:
      prepare_word_response(chip, chip->asc_target);
      break;

    case CMD_GET_ASC_INITIAL_PERIOD:
      prepare_word_response(chip, chip->asc_initial_period);
      break;

    case CMD_GET_ASC_STANDARD_PERIOD:
      prepare_word_response(chip, chip->asc_standard_period);
      break;

    case CMD_PERSIST_SETTINGS:
      chip->stored_temp_offset = chip->temperature_offset_raw;
      chip->stored_altitude = chip->sensor_altitude;
      chip->stored_asc = chip->asc_enabled;
      chip->stored_asc_target = chip->asc_target;
      chip->stored_asc_initial_period = chip->asc_initial_period;
      chip->stored_asc_standard_period = chip->asc_standard_period;
      chip->has_persisted = true;
      chip->eeprom_write_count++;
      if (chip->eeprom_write_count > 2000) {
        printf("WARNING: EEPROM write count %u exceeds 2000-cycle limit!\n", chip->eeprom_write_count);
      }
      chip->busy_until = get_sim_nanos() + 800000000ULL; // 800ms
      break;

    case CMD_SELF_TEST:
      prepare_word_response(chip, 0x0000); // no malfunction
      chip->busy_until = get_sim_nanos() + 10000000000ULL; // 10s
      break;

    case CMD_FACTORY_RESET:
      reset_to_defaults(chip);
      chip->stored_temp_offset = DEFAULT_TEMP_OFFSET;
      chip->stored_altitude = DEFAULT_ALTITUDE;
      chip->stored_asc = DEFAULT_ASC;
      chip->stored_asc_target = DEFAULT_ASC_TARGET;
      chip->stored_asc_initial_period = DEFAULT_ASC_INITIAL_PERIOD;
      chip->stored_asc_standard_period = DEFAULT_ASC_STANDARD_PERIOD;
      chip->has_persisted = false;
      chip->has_measured = false;
      chip->busy_until = get_sim_nanos() + 1200000000ULL; // 1200ms
      break;

    case CMD_REINIT:
      if (chip->has_persisted) {
        chip->temperature_offset_raw = chip->stored_temp_offset;
        chip->sensor_altitude = chip->stored_altitude;
        chip->asc_enabled = chip->stored_asc;
        chip->asc_target = chip->stored_asc_target;
        chip->asc_initial_period = chip->stored_asc_initial_period;
        chip->asc_standard_period = chip->stored_asc_standard_period;
      } else {
        reset_to_defaults(chip);
      }
      chip->busy_until = get_sim_nanos() + 30000000ULL; // 30ms
      break;

    case CMD_MEASURE_SINGLE_SHOT:
      chip->state = STATE_MEASURING;
      chip->data_ready = false;
      chip->single_shot = true;
      chip->single_shot_rht_only = false;
      chip->low_power_mode = false;
      timer_start(chip->measurement_timer, 5000000, false);
      chip->busy_until = get_sim_nanos() + 5000000000ULL;
      break;

    case CMD_MEASURE_SINGLE_SHOT_RHT:
      chip->state = STATE_MEASURING;
      chip->data_ready = false;
      chip->single_shot = true;
      chip->single_shot_rht_only = true;
      chip->low_power_mode = false;
      timer_start(chip->measurement_timer, 50000, false);
      chip->busy_until = get_sim_nanos() + 50000000ULL;
      break;

    case CMD_POWER_DOWN:
      chip->state = STATE_POWERED_DOWN;
      break;

    case CMD_WAKE_UP:
      chip->state = STATE_IDLE;
      chip->busy_until = get_sim_nanos() + 30000000ULL; // 30ms
      break;

    default:
      break;
  }
}

static void process_payload_command(chip_state_t *chip, uint16_t cmd, uint16_t value) {
  if (!command_allowed(chip, cmd)) return;

  chip->response_len = 0;
  chip->response_idx = 0;

  switch (cmd) {
    case CMD_SET_TEMP_OFFSET:
      chip->temperature_offset_raw = value;
      break;

    case CMD_SET_ALTITUDE:
      chip->sensor_altitude = value;
      break;

    case CMD_SET_AMBIENT_PRESSURE:
      chip->ambient_pressure = value;
      break;

    case CMD_SET_ASC:
      chip->asc_enabled = (value != 0);
      break;

    case CMD_SET_ASC_TARGET:
      chip->asc_target = value;
      break;

    case CMD_SET_ASC_INITIAL_PERIOD:
      chip->asc_initial_period = value;
      break;

    case CMD_SET_ASC_STANDARD_PERIOD:
      chip->asc_standard_period = value;
      break;

    case CMD_FORCED_RECAL:
      if (!chip->has_measured) {
        prepare_word_response(chip, 0xFFFF);
      } else {
        prepare_word_response(chip, 0x8000);
      }
      chip->busy_until = get_sim_nanos() + 400000000ULL; // 400ms
      break;

    default:
      break;
  }
}

static void measurement_timer_callback(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  chip->data_ready = true;
  chip->has_measured = true;
  if (chip->single_shot) return; // no repeating timer for single-shot
  if (!chip->first_measurement_done) {
    chip->first_measurement_done = true;
    uint32_t interval = chip->low_power_mode ? 30000000 : 5000000;
    timer_start(chip->measurement_timer, interval, true);
  }
}

static bool on_i2c_connect(void *user_data, uint32_t address, bool connect) {
  chip_state_t *chip = (chip_state_t *)user_data;
  (void)address;
  if (connect) {
    // NACK while busy
    if (chip->busy_until > 0 && get_sim_nanos() < chip->busy_until) {
      return false;
    }
    chip->write_count = 0;
  }
  return true;
}

static uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (chip->state == STATE_POWERED_DOWN) {
    return 0xFF;
  }
  if (chip->response_idx < chip->response_len) {
    return chip->response[chip->response_idx++];
  }
  return 0xFF;
}

static bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  if (chip->write_count == 0) {
    // New command starting: clear any previous response
    chip->response_len = 0;
    chip->response_idx = 0;
  }

  if (chip->write_count < 5) {
    chip->write_buf[chip->write_count] = data;
    chip->write_count++;
  }

  if (chip->write_count == 2) {
    chip->current_cmd = (uint16_t)((uint16_t)chip->write_buf[0] << 8) | chip->write_buf[1];
    if (!command_has_payload(chip->current_cmd)) {
      process_command(chip, chip->current_cmd);
    }
  } else if (chip->write_count == 5) {
    // Payload complete: data[2..3] + CRC[4]
    uint8_t expected_crc = crc8(&chip->write_buf[2], 2);
    if (expected_crc == chip->write_buf[4]) {
      uint16_t value = (uint16_t)((uint16_t)chip->write_buf[2] << 8) | chip->write_buf[3];
      process_payload_command(chip, chip->current_cmd, value);
    }
    // Bad CRC: silently ignored (matches real chip)
  }

  return true; // ACK all bytes
}

static void on_i2c_disconnect(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (chip->write_count == 2 && chip->current_cmd == CMD_SET_AMBIENT_PRESSURE) {
    if (command_allowed(chip, chip->current_cmd)) {
      prepare_word_response(chip, chip->ambient_pressure);
    }
  }
  chip->write_count = 0;
}

void chip_init(void) {
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));

  chip->variant_attr = attr_init("variant", 1); // default SCD41
  chip->serial0_attr = attr_init("serial0", 0x0001);
  chip->serial1_attr = attr_init("serial1", 0x0002);
  chip->serial2_attr = attr_init("serial2", 0x0003);
  chip->co2_attr = attr_init_float("co2", 800.0f);
  chip->temp_attr = attr_init_float("temperature", 25.0f);
  chip->hum_attr = attr_init_float("humidity", 50.0f);

  reset_to_defaults(chip);
  chip->has_persisted = false;

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
