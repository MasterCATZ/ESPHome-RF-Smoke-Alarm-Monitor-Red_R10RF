#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <string.h>

// =====================================================
// Red R10RF CC1101 receive only detector + optional PIR + full frame capture
//
// ESP32 S3 WROOM
// CC1101 pins:
//   SCK  15
//   MISO 16
//   MOSI 17
//   CS   18
//   GDO0 7   raw serial data
//   GDO2 6   carrier sense
// PIR pin (optional):
//   GPIO 4  (configurable via PIR_PIN)
// =====================================================

#define CC1101_IOCFG0 0x02
#define CC1101_IOCFG2 0x00

#define GDO_CARRIER_SENSE 0x0E
#define GDO_SERIAL_DATA   0x0D

static constexpr int PIN_SCK  = 15;
static constexpr int PIN_MISO = 16;
static constexpr int PIN_MOSI = 17;
static constexpr int PIN_CS   = 18;
static constexpr int PIN_GDO0 = 7;
static constexpr int PIN_GDO2 = 6;

// PIR pin - can be overridden by defining PIR_PIN before include
#ifndef PIR_PIN
#define PIR_PIN 4
#endif

static constexpr float RF_FREQ      = 433.92;
static constexpr float RF_DATARATE  = 9.6;
static constexpr float RF_DEVIATION = 47.6;
static constexpr float RF_RXBW      = 203.1;

static constexpr uint16_t MAX_EDGES = 12000;
static constexpr uint16_t MIN_EDGES_TO_PROCESS = 80;
static constexpr uint32_t TRAIN_END_US = 20000;
static constexpr uint32_t MAX_CAPTURE_US = 2500000;

static constexpr float BIT_US = 104.0;
static constexpr uint32_t GLITCH_REJECT_US = 35;
static constexpr uint32_t FRAME_GAP_US = 500;
static constexpr uint8_t MAX_RUN_BITS = 32;
static constexpr uint16_t MAX_FRAME_BITS = 512;

static constexpr uint8_t MAX_KNOWN_CODES = 16;
static constexpr uint8_t MAX_CODE_LEN = 32;
static constexpr uint8_t FRAME_TEXT_LEN = 64;

// Capture log settings
static constexpr uint8_t MAX_CAPTURE_LOG = 20;
static constexpr uint8_t CAPTURE_LOG_LEN = 80;

static volatile uint32_t edge_time[MAX_EDGES];
static volatile uint8_t edge_level[MAX_EDGES];
static volatile uint16_t edge_count = 0;
static volatile bool edge_overflow = false;

static volatile bool carrier_active = false;
static volatile bool capture_had_carrier = false;
static volatile uint32_t capture_start_us = 0;
static volatile uint32_t last_carrier_change_us = 0;

static bool alarm_detected = false;
static bool alarm_latched = false;
static bool unknown_frame_seen = false;

static int last_rssi = -120;
static char last_frame[FRAME_TEXT_LEN] = "";
static char last_raw_frame[FRAME_TEXT_LEN] = "";
static char last_unknown_frame[FRAME_TEXT_LEN] = "";

// Full frame capture log (circular buffer)
static char capture_log[MAX_CAPTURE_LOG][CAPTURE_LOG_LEN];
static uint8_t capture_log_head = 0;
static uint8_t capture_log_count = 0;
static bool capture_all_frames = false;

// PIR state
static bool pir_last_state = false;
static bool pir_latched = false;
static uint32_t pir_last_change_ms = 0;

static char known_codes[MAX_KNOWN_CODES][MAX_CODE_LEN];
static uint8_t known_code_count = 0;

static uint32_t last_rx_reset_ms = 0;

// =====================================================
// User supplied code list
// =====================================================

void r10rf_set_known_codes(const char *csv) {
  known_code_count = 0;

  for (uint8_t i = 0; i < MAX_KNOWN_CODES; i++) {
    known_codes[i][0] = '\0';
  }

  if (csv == nullptr || csv[0] == '\0') {
    return;
  }

  char buffer[256];
  strncpy(buffer, csv, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  char *token = strtok(buffer, ",");

  while (token != nullptr && known_code_count < MAX_KNOWN_CODES) {
    while (*token == ' ' || *token == '\t' || *token == '\n' || *token == '\r') {
      token++;
    }

    char *end = token + strlen(token);
    while (end > token && (*(end - 1) == ' ' || *(end - 1) == '\t' || *(end - 1) == '\n' || *(end - 1) == '\r')) {
      *(end - 1) = '\0';
      end--;
    }

    if (token[0] != '\0') {
      strncpy(known_codes[known_code_count], token, MAX_CODE_LEN - 1);
      known_codes[known_code_count][MAX_CODE_LEN - 1] = '\0';
      known_code_count++;
    }

    token = strtok(nullptr, ",");
  }
}

// =====================================================
// Interrupts
// =====================================================

void IRAM_ATTR r10rf_gdo0_isr() {
  if (!carrier_active) return;

  if (edge_count < MAX_EDGES) {
    edge_time[edge_count] = micros();
    edge_level[edge_count] = digitalRead(PIN_GDO0);
    edge_count++;
  } else {
    edge_overflow = true;
  }
}

void IRAM_ATTR r10rf_gdo2_isr() {
  bool state = digitalRead(PIN_GDO2);
  carrier_active = state;
  last_carrier_change_us = micros();

  if (state && !capture_had_carrier) {
    capture_had_carrier = true;
    capture_start_us = micros();
  }
}

// =====================================================
// Decode helpers
// =====================================================

static uint8_t r10rf_duration_to_bits(uint32_t dt) {
  if (dt < GLITCH_REJECT_US) return 0;

  uint8_t bits = (uint8_t)((dt + (BIT_US / 2.0)) / BIT_US);

  if (bits < 1) bits = 1;
  if (bits > MAX_RUN_BITS) bits = MAX_RUN_BITS;

  return bits;
}

static char r10rf_hex_char(uint8_t value) {
  return value < 10 ? (char)('0' + value) : (char)('a' + value - 10);
}

static void r10rf_bits_to_hex(const char *bits, char *out_hex, size_t out_size, bool invert) {
  uint8_t nibble = 0;
  uint8_t nibble_count = 0;
  size_t out_index = 0;

  for (uint16_t i = 0; bits[i] != '\0'; i++) {
    char c = bits[i];

    if (c != '0' && c != '1') continue;

    if (invert) c = (c == '1') ? '0' : '1';

    nibble <<= 1;
    if (c == '1') nibble |= 1;
    nibble_count++;

    if (nibble_count == 4) {
      if (out_index < out_size - 1) {
        out_hex[out_index++] = r10rf_hex_char(nibble);
      }

      nibble = 0;
      nibble_count = 0;
    }
  }

  if (nibble_count > 0) {
    nibble <<= (4 - nibble_count);

    if (out_index < out_size - 1) {
      out_hex[out_index++] = r10rf_hex_char(nibble);
    }
  }

  out_hex[out_index] = '\0';
}

static bool r10rf_is_alarm_frame(const char *hex) {
  if (hex == nullptr || hex[0] == '\0') return false;

  for (uint8_t i = 0; i < known_code_count; i++) {
    if (known_codes[i][0] == '\0') continue;

    if (strstr(hex, known_codes[i]) != nullptr) {
      return true;
    }
  }

  return false;
}

static void r10rf_store_raw_frame(const char *hex) {
  if (hex == nullptr || hex[0] == '\0') return;

  strncpy(last_raw_frame, hex, sizeof(last_raw_frame) - 1);
  last_raw_frame[sizeof(last_raw_frame) - 1] = '\0';
}

static void r10rf_store_unknown_frame(const char *hex) {
  if (hex == nullptr || hex[0] == '\0') return;

  strncpy(last_unknown_frame, hex, sizeof(last_unknown_frame) - 1);
  last_unknown_frame[sizeof(last_unknown_frame) - 1] = '\0';
  unknown_frame_seen = true;
}

// Add frame to circular capture log with timestamp
static void r10rf_log_capture(const char *hex, bool is_alarm, bool is_unknown) {
  if (!capture_all_frames) return;
  if (hex == nullptr || hex[0] == '\0') return;

  char log_entry[CAPTURE_LOG_LEN];
  uint32_t now_ms = millis();
  uint32_t secs = now_ms / 1000;
  uint32_t ms = now_ms % 1000;

  char type_char = is_alarm ? 'A' : (is_unknown ? 'U' : 'R');
  snprintf(log_entry, sizeof(log_entry), "[%lu.%03lu] %c %s", secs, ms, type_char, hex);

  strncpy(capture_log[capture_log_head], log_entry, CAPTURE_LOG_LEN - 1);
  capture_log[capture_log_head][CAPTURE_LOG_LEN - 1] = '\0';

  capture_log_head = (capture_log_head + 1) % MAX_CAPTURE_LOG;
  if (capture_log_count < MAX_CAPTURE_LOG) capture_log_count++;
}

static bool r10rf_process_frame_bits(const char *frame_bits) {
  char normal_hex[FRAME_TEXT_LEN];
  char inverted_hex[FRAME_TEXT_LEN];

  r10rf_bits_to_hex(frame_bits, normal_hex, sizeof(normal_hex), false);
  r10rf_bits_to_hex(frame_bits, inverted_hex, sizeof(inverted_hex), true);

  r10rf_store_raw_frame(normal_hex);

  bool is_alarm = false;
  if (r10rf_is_alarm_frame(normal_hex)) {
    strncpy(last_frame, normal_hex, sizeof(last_frame) - 1);
    last_frame[sizeof(last_frame) - 1] = '\0';
    is_alarm = true;
    r10rf_log_capture(normal_hex, true, false);
    return true;
  }

  if (r10rf_is_alarm_frame(inverted_hex)) {
    strncpy(last_frame, inverted_hex, sizeof(last_frame) - 1);
    last_frame[sizeof(last_frame) - 1] = '\0';
    is_alarm = true;
    r10rf_log_capture(inverted_hex, true, false);
    return true;
  }

  r10rf_store_unknown_frame(normal_hex);
  r10rf_log_capture(normal_hex, false, true);
  return false;
}

static bool r10rf_decode_edges(uint32_t *times, uint8_t *levels, uint16_t count) {
  bool found_alarm = false;
  bool frame_open = false;

  char frame_bits[MAX_FRAME_BITS];
  uint16_t frame_bit_count = 0;

  for (uint16_t i = 1; i < count; i++) {
    uint32_t dt = times[i] - times[i - 1];

    if (dt < GLITCH_REJECT_US) continue;

    if (dt > FRAME_GAP_US) {
      if (frame_open && frame_bit_count > 0) {
        frame_bits[frame_bit_count] = '\0';

        if (r10rf_process_frame_bits(frame_bits)) {
          found_alarm = true;
        }

        frame_open = false;
        frame_bit_count = 0;
      }

      continue;
    }

    uint8_t run_bits = r10rf_duration_to_bits(dt);
    if (run_bits == 0) continue;

    uint8_t bit_value = levels[i - 1];

    if (!frame_open) {
      frame_open = true;
      frame_bit_count = 0;
    }

    for (uint8_t b = 0; b < run_bits; b++) {
      if (frame_bit_count < MAX_FRAME_BITS - 1) {
        frame_bits[frame_bit_count++] = bit_value ? '1' : '0';
      }
    }
  }

  if (frame_open && frame_bit_count > 0) {
    frame_bits[frame_bit_count] = '\0';

    if (r10rf_process_frame_bits(frame_bits)) {
      found_alarm = true;
    }
  }

  // Log non-alarm frames too if capture_all is enabled
  if (capture_all_frames && frame_open && frame_bit_count > 0 && !found_alarm) {
    char normal_hex[FRAME_TEXT_LEN];
    r10rf_bits_to_hex(frame_bits, normal_hex, sizeof(normal_hex), false);
    r10rf_log_capture(normal_hex, false, false);
  }

  return found_alarm;
}

// =====================================================
// CC1101 setup
// =====================================================

static void r10rf_configure_radio() {
  ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  ELECHOUSE_cc1101.Init();

  ELECHOUSE_cc1101.setCCMode(1);
  ELECHOUSE_cc1101.setModulation(0);  // 0 = 2 FSK
  ELECHOUSE_cc1101.setMHZ(RF_FREQ);
  ELECHOUSE_cc1101.setChannel(0);
  ELECHOUSE_cc1101.setChsp(199.95);

  ELECHOUSE_cc1101.setDRate(RF_DATARATE);
  ELECHOUSE_cc1101.setDeviation(RF_DEVIATION);
  ELECHOUSE_cc1101.setRxBW(RF_RXBW);

  ELECHOUSE_cc1101.setSyncMode(0);
  ELECHOUSE_cc1101.setSyncWord(0, 0);
  ELECHOUSE_cc1101.setAdrChk(0);
  ELECHOUSE_cc1101.setAddr(0);
  ELECHOUSE_cc1101.setWhiteData(0);
  ELECHOUSE_cc1101.setCrc(0);
  ELECHOUSE_cc1101.setCRC_AF(0);
  ELECHOUSE_cc1101.setManchester(0);
  ELECHOUSE_cc1101.setFEC(0);
  ELECHOUSE_cc1101.setPRE(0);
  ELECHOUSE_cc1101.setPQT(0);
  ELECHOUSE_cc1101.setDcFilterOff(0);
  ELECHOUSE_cc1101.setAppendStatus(0);

  ELECHOUSE_cc1101.setPktFormat(3);       // asynchronous serial
  ELECHOUSE_cc1101.setLengthConfig(2);    // infinite packet mode
  ELECHOUSE_cc1101.setPacketLength(255);
  ELECHOUSE_cc1101.setPA(0);

  ELECHOUSE_cc1101.setGDO0(PIN_GDO0);

  ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, GDO_SERIAL_DATA);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG2, GDO_CARRIER_SENSE);

  ELECHOUSE_cc1101.SetRx();
}

// PIR helper functions
static inline bool r10rf_pir_read() {
  return digitalRead(PIR_PIN);
}

static inline bool r10rf_pir_has_motion() {
  return pir_last_state;
}

static inline bool r10rf_pir_is_latched() {
  return pir_latched;
}

static inline void r10rf_pir_acknowledge() {
  pir_latched = false;
}

static inline void r10rf_pir_clear_motion() {
  pir_last_state = false;
}

// Capture mode control
void r10rf_capture_all_enable(bool enable) {
  capture_all_frames = enable;
  if (enable) {
    capture_log_head = 0;
    capture_log_count = 0;
    for (uint8_t i = 0; i < MAX_CAPTURE_LOG; i++) {
      capture_log[i][0] = '\0';
    }
  }
}

bool r10rf_capture_all_enabled() {
  return capture_all_frames;
}

uint8_t r10rf_capture_log_count() {
  return capture_log_count;
}

const char *r10rf_capture_log_get(uint8_t index) {
  if (index >= capture_log_count) return nullptr;
  uint8_t idx = (capture_log_head + index) % MAX_CAPTURE_LOG;
  return capture_log[idx];
}

void r10rf_setup() {
  pinMode(PIN_GDO0, INPUT);
  pinMode(PIN_GDO2, INPUT);
  pinMode(PIR_PIN, INPUT);

  r10rf_configure_radio();

  attachInterrupt(digitalPinToInterrupt(PIN_GDO0), r10rf_gdo0_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_GDO2), r10rf_gdo2_isr, CHANGE);

  edge_count = 0;
  edge_overflow = false;
  carrier_active = digitalRead(PIN_GDO2);
  capture_had_carrier = false;
  capture_start_us = micros();
  last_carrier_change_us = micros();

  // Initialize PIR state
  pir_last_state = digitalRead(PIR_PIN);
  pir_latched = false;
  pir_last_change_ms = millis();

  // Initialize capture log
  capture_all_frames = false;
  capture_log_head = 0;
  capture_log_count = 0;
  for (uint8_t i = 0; i < MAX_CAPTURE_LOG; i++) {
    capture_log[i][0] = '\0';
  }
}

// =====================================================
// Runtime
// =====================================================

void r10rf_loop() {
  uint32_t now = micros();
  uint32_t now_ms = millis();

  bool should_process = false;

  noInterrupts();

  bool had_carrier = capture_had_carrier;
  bool active = carrier_active;
  uint32_t since_carrier_change = now - last_carrier_change_us;
  uint16_t count = edge_count;
  uint32_t started_at = capture_start_us;

  interrupts();

  bool timed_out = had_carrier && ((now - started_at) > MAX_CAPTURE_US);

  if ((had_carrier && !active && since_carrier_change > TRAIN_END_US && count >= MIN_EDGES_TO_PROCESS) ||
      (timed_out && count >= MIN_EDGES_TO_PROCESS)) {
    should_process = true;
  }

  // PIR polling (non-interrupt, 50ms interval is fine)
  bool pir_now = r10rf_pir_read();
  if (pir_now != pir_last_state) {
    pir_last_state = pir_now;
    pir_last_change_ms = now_ms;
    if (pir_now) {
      pir_latched = true;
    }
  }

  if (!should_process) {
    if (millis() - last_rx_reset_ms > 30000) {
      last_rx_reset_ms = millis();
      ELECHOUSE_cc1101.SetRx();
    }
    return;
  }

  static uint32_t times_copy[MAX_EDGES];
  static uint8_t levels_copy[MAX_EDGES];

  noInterrupts();

  count = edge_count;
  bool overflow = edge_overflow;

  if (count > MAX_EDGES) count = MAX_EDGES;

  for (uint16_t i = 0; i < count; i++) {
    times_copy[i] = edge_time[i];
    levels_copy[i] = edge_level[i];
  }

  edge_count = 0;
  edge_overflow = false;
  capture_had_carrier = false;
  carrier_active = digitalRead(PIN_GDO2);
  capture_start_us = micros();
  last_carrier_change_us = micros();

  interrupts();

  last_rssi = ELECHOUSE_cc1101.getRssi();

  if (!overflow && count >= MIN_EDGES_TO_PROCESS) {
    if (r10rf_decode_edges(times_copy, levels_copy, count)) {
      alarm_detected = true;
      alarm_latched = true;
    }
  }

  ELECHOUSE_cc1101.SetRx();
}

bool r10rf_has_alarm() {
  return alarm_detected;
}

void r10rf_clear_alarm() {
  // Clears only the short event flag. The latched alarm remains on until acknowledged.
  alarm_detected = false;
}

bool r10rf_alarm_is_latched() {
  return alarm_latched;
}

void r10rf_acknowledge_alarm() {
  alarm_detected = false;
  alarm_latched = false;
}

const char *r10rf_get_last_frame() {
  return last_frame;
}

const char *r10rf_get_last_raw_frame() {
  return last_raw_frame;
}

bool r10rf_has_unknown_frame() {
  return unknown_frame_seen;
}

const char *r10rf_get_last_unknown_frame() {
  return last_unknown_frame;
}

void r10rf_clear_unknown_frame() {
  unknown_frame_seen = false;
}

int r10rf_last_rssi() {
  return last_rssi;
}