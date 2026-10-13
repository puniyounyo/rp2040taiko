/*
 * SNES(SFC)純正コントローラー + ピエゾ4ch → Nintendo Switch「ポッ拳コントローラー」偽装
 * Board: Waveshare RP2040-Zero (NeoPixel on GP16)
 * Core : Earle Philhower "arduino-pico"
 * Tools > USB Stack : "Adafruit TinyUSB" を選択すること
 */

#include "Adafruit_TinyUSB.h"
#include "hardware/adc.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <string.h>
#include <stdlib.h>

#define OLED_SDA 4
#define OLED_SCL 5
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 16
#endif

#define NUM_PIXELS 1
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

#define SWITCH_VID  0x0F0D
#define SWITCH_PID  0x0092

enum { REPORT_ID_KEYBOARD = 1 };
uint8_t const desc_hid_report_keyboard[] = {
  TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD))
};

uint8_t key_map[4] = {HID_KEY_D, HID_KEY_F, HID_KEY_J, HID_KEY_K};
bool switchMode = true;
bool oled_ok = false;
bool settingMode = false;
bool autoCalibMode = false; 
int currentSetting = 0;
bool raw_debug_mode = false;

// 12項目の精鋭パラメータ
#define KANDO_COUNT 12
int kando[KANDO_COUNT] = {
    80,  // [0] L_EDGE
    160, // [1] L_FACE
    160, // [2] R_FACE
    80,  // [3] R_EDGE
    30,  // [4] FACE_INTV
    12,  // [5] EDGE_INTV
    35,  // [6] CROSS_IG (異系統ガード)
    0,   // [7] HOLD_MS
    0,   // [8] FACE_SGL
    0,   // [9] EDGE_SGL
    40,  // [10] FACE_LR (左右面ガード)
    40   // [11] EDGE_LR (左右縁ガード)
};

const char *kandoNames[KANDO_COUNT] = {
    "L_EDGE", "L_FACE", "R_FACE", "R_EDGE", "FACE_INTV",
    "EDGE_INTV", "CROSS_IG", "HOLD_MS", "FACE_SGL", "EDGE_SGL",
    "FACE_LR", "EDGE_LR"
};
#define EEPROM_SIZE 16

#define PIN_LATCH 0
#define PIN_CLOCK 1
#define PIN_DATA  2

#define SNES_B      0
#define SNES_Y      1
#define SNES_SELECT 2
#define SNES_START  3
#define SNES_UP     4
#define SNES_DOWN   5
#define SNES_LEFT   6
#define SNES_RIGHT  7
#define SNES_A      8
#define SNES_X      9
#define SNES_L      10
#define SNES_R      11

long int result_ac[4] = {0, 0, 0, 0};
int32_t last_deltas[4] = {0, 0, 0, 0};
uint16_t delta_max[4] = {0};

#define CAL_ROUTE_COUNT 12

struct CalibrationProfile {
  uint16_t threshold[4];
  uint16_t self_face_b;   
  uint16_t self_edge_c;   
  uint16_t self_face_a;   
  uint16_t self_edge_a;   
  uint16_t pair_delay_ms[CAL_ROUTE_COUNT];  
};

static inline uint8_t cal_route_index(uint8_t source, uint8_t target) {
  if (source >= 4 || target >= 4 || source == target) return 0xFF;
  return (uint8_t)(source * 3 + target - (target > source ? 1 : 0));
}

static inline bool cal_same_type(uint8_t a, uint8_t b) {
  bool a_face = (a == 1 || a == 2);
  bool b_face = (b == 1 || b == 2);
  return a_face == b_face;
}

void build_profile_from_kando(CalibrationProfile &p) {
  for (int i = 0; i < 4; i++) p.threshold[i] = (uint16_t)kando[i];
  p.self_face_b = (uint16_t)kando[4];
  p.self_edge_c = (uint16_t)kando[5];
  p.self_face_a = (uint16_t)kando[8];
  p.self_edge_a = (uint16_t)kando[9];
  memset(p.pair_delay_ms, 0, sizeof(p.pair_delay_ms));

  for (uint8_t s = 0; s < 4; s++) {
    for (uint8_t t = 0; t < 4; t++) {
      uint8_t route = cal_route_index(s, t);
      if (route == 0xFF) continue;
      
      if (!cal_same_type(s, t)) {
        p.pair_delay_ms[route] = (uint16_t)kando[6]; // CROSS_IG
      } else {
        if (s != t) {
          bool is_face = (s == 1 || s == 2);
          if (is_face) {
            p.pair_delay_ms[route] = (uint16_t)kando[10]; // FACE_LR
          } else {
            p.pair_delay_ms[route] = (uint16_t)kando[11]; // EDGE_LR
          }
        }
      }
    }
  }
}

class PairDelayGate {
public:
  void reset() {
    memset(last_hit_ms_, 0, sizeof(last_hit_ms_));
    last_face_ms_ = last_edge_ms_ = 0;
    seen_mask_ = 0;
  }

  bool accept(uint8_t target, int32_t delta, uint32_t now_ms, const CalibrationProfile &p) {
    if (target >= 4 || delta <= (int32_t)p.threshold[target]) return false;
    bool face = (target == 1 || target == 2);
    uint32_t type_elapsed = now_ms - (face ? last_face_ms_ : last_edge_ms_);
    uint16_t type_delay = face ? p.self_face_b : p.self_edge_c;
    if (type_elapsed < type_delay) return false;

    uint32_t self_elapsed = now_ms - last_hit_ms_[target];
    uint16_t self_delay = face ? p.self_face_a : p.self_edge_a;
    if (self_elapsed <= self_delay) return false;

    for (uint8_t source = 0; source < 4; source++) {
      if (source == target || (seen_mask_ & (1u << source)) == 0) continue;
      uint8_t route = cal_route_index(source, target);
      uint32_t elapsed = now_ms - last_hit_ms_[source];
      uint16_t delay = p.pair_delay_ms[route];
      if (cal_same_type(source, target)) {
        if (elapsed < delay) return false;
      } else if (elapsed <= delay) {
        return false;
      }
    }

    last_hit_ms_[target] = now_ms;
    if (face) last_face_ms_ = now_ms; else last_edge_ms_ = now_ms;
    seen_mask_ |= (uint8_t)(1u << target);
    return true;
  }

private:
    uint32_t last_hit_ms_[4] = {0};
    uint32_t last_face_ms_ = 0;
    uint32_t last_edge_ms_ = 0;
    uint8_t seen_mask_ = 0;
};

PairDelayGate piezo_gate;
CalibrationProfile piezo_profile;

int calibTargetSensor = 0;      
int calibHitCount = 0;          
int32_t calibSensorMax[4][20];  
int32_t calibCrosstalkMax[4];  

uint8_t const desc_hid_report[] = {
  0x05, 0x01,      0x09, 0x05,      0xA1, 0x01,      
    0x15, 0x00,      0x25, 0x01,      0x35, 0x00,      0x45, 0x01,      
    0x75, 0x01,      0x95, 0x0D,      0x05, 0x09,      0x19, 0x01,      
    0x29, 0x0D,      0x81, 0x02,      0x75, 0x03,      0x95, 0x01,      
    0x81, 0x03,      0x05, 0x01,      0x25, 0x07,      0x46, 0x3B, 0x01,
    0x75, 0x04,      0x95, 0x01,      0x65, 0x14,      0x09, 0x39,      
    0x81, 0x42,      0x75, 0x04,      0x95, 0x01,      0x81, 0x03,      
    0x15, 0x00,      0x26, 0xFF, 0x0F,0x75, 0x10,      0x95, 0x04,      
    0x09, 0x30,      0x09, 0x31,      0x09, 0x32,      0x09, 0x35,      
    0x81, 0x02,      0xC0               
};

Adafruit_USBD_HID usb_hid;

typedef struct __attribute__((packed)) {
  uint16_t buttons; 
  uint8_t  hat;     
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint16_t rz;
} pokken_report_t;

#define POKKEN_Y      0 
#define POKKEN_B      1 
#define POKKEN_A      2 
#define POKKEN_X      3 
#define POKKEN_L      4 
#define POKKEN_R      5 
#define POKKEN_ZL     6 
#define POKKEN_ZR     7 
#define POKKEN_MINUS  8 
#define POKKEN_PLUS   9 
#define POKKEN_LCLICK 10 
#define POKKEN_RCLICK 11 
#define POKKEN_HOME   12 

uint16_t readSNES();
void loadSettings();
void saveSettings();

void setup() {
  pixels.begin();
  pixels.setBrightness(30); 
  pixels.show();

  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_DATA, INPUT_PULLUP);
  digitalWrite(PIN_LATCH, LOW);
  digitalWrite(PIN_CLOCK, HIGH);

  delay(5);
  bool boot_start = false;
  for (int i = 0; i < 3; i++) {
    uint16_t s = readSNES();
    if (s & (1 << SNES_START)) { boot_start = true; break; }
    delay(3);
  }
  switchMode = !boot_start;

  if (switchMode) {
    Serial.end();
    USBDevice.setID(SWITCH_VID, SWITCH_PID);
    USBDevice.setManufacturerDescriptor("HORI CO., LTD.");
    USBDevice.setProductDescriptor("POKKEN CONTROLLER");
    usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.show();
  } else {
    USBDevice.setID(0xCAFE, 0x4004);
    usb_hid.setReportDescriptor(desc_hid_report_keyboard, sizeof(desc_hid_report_keyboard));
    
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();
  }

  usb_hid.setPollInterval(2);
  usb_hid.begin();

  if (switchMode) {
    while (!USBDevice.mounted()) { delay(10); }
    delay(500);
  } else {
    Serial.begin(115200);
    delay(500);
  }

  adc_init();
  adc_gpio_init(26);
  adc_gpio_init(27);
  adc_gpio_init(28);
  adc_gpio_init(29);

  EEPROM.begin(EEPROM_SIZE);
  loadSettings();
  piezo_gate.reset();

  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  Wire.begin();
  Wire.beginTransmission(0x3C);
  oled_ok = (Wire.endTransmission() == 0);
  if (oled_ok) {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println(switchMode ? "SW MODE" : "PC MODE");
    display.display();
  }
}

uint16_t readSNES() {
  uint16_t state = 0;
  digitalWrite(PIN_LATCH, HIGH);
  delayMicroseconds(12);
  digitalWrite(PIN_LATCH, LOW);
  delayMicroseconds(6);

  for (int i = 0; i < 16; i++) {
    if (digitalRead(PIN_DATA) == LOW) { state |= (1 << i); }
    digitalWrite(PIN_CLOCK, LOW);
    delayMicroseconds(6);
    digitalWrite(PIN_CLOCK, HIGH);
    delayMicroseconds(6);
  }
  return state;
}

uint8_t calcHat(bool up, bool down, bool left, bool right) {
  if (up && right) return 1;
  if (down && right) return 3;
  if (down && left) return 5;
  if (up && left) return 7;
  if (up) return 0;
  if (right) return 2;
  if (down) return 4;
  if (left) return 6;
  return 8; 
}

uint8_t read_piezo_hits() {
  static uint64_t last_sample_us = 0;
  uint64_t now_us = to_us_since_boot(get_absolute_time());
  if (now_us - last_sample_us < 10) return 0;
  last_sample_us = now_us;

  build_profile_from_kando(piezo_profile); 
  uint32_t main_timer = (uint32_t)millis();
  uint8_t hits = 0;

  for (int i = 0; i < 4; i++) {
    adc_select_input(3 - i); 
    long int result = adc_read();
    int32_t delta = (int32_t)(result - result_ac[i]);
    last_deltas[i] = delta;

    if (delta > (long int)delta_max[i]) delta_max[i] = (uint16_t)delta;

    if (autoCalibMode) {
      if (i == calibTargetSensor && delta > 20) {
        if (calibHitCount < 20) {
          calibSensorMax[calibTargetSensor][calibHitCount] = delta;
          Serial.print("[CALIB] Target: ");
          Serial.print(kandoNames[calibTargetSensor]);
          Serial.print(" | Hit #");
          Serial.print(calibHitCount + 1);
          Serial.print(" | Delta: ");
          Serial.println(delta);
          calibHitCount++;
        }
      }
      if (calibHitCount > 0 && calibHitCount <= 20 && i != calibTargetSensor) {
        if (delta > calibCrosstalkMax[calibTargetSensor]) {
          calibCrosstalkMax[calibTargetSensor] = delta;
        }
      }
    } else {
      if (piezo_gate.accept((uint8_t)i, delta, main_timer, piezo_profile)) {
        hits |= (1 << i);
      }
    }
    result_ac[i] = result;
  }
  return hits;
}

void displaySetting() {
  if (!oled_ok) return;
  display.clearDisplay();
  
  if (autoCalibMode) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("--- GOD CALIB ---");
    display.setCursor(0, 16);
    display.print("Target: ");
    display.println(kandoNames[calibTargetSensor]);
    display.setTextSize(2);
    display.setCursor(0, 36);
    display.print("Hit:");
    display.print(calibHitCount);
    display.println("/20");
  } else {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("--- SETTING ---");
    display.setCursor(0, 16);
    display.print(kandoNames[currentSetting]);
    display.setTextSize(2);
    display.setCursor(0, 36);
    display.print(">");
    display.println(kando[currentSetting]);
  }
  display.display();
}

void saveSettings() {
  for (int i = 0; i < KANDO_COUNT; i++) EEPROM.write(i, (uint8_t)kando[i]);
  EEPROM.commit();
}

void loadSettings() {
  for (int i = 0; i < KANDO_COUNT; i++) {
    uint8_t v = EEPROM.read(i);
    if (v != 0xFF) kando[i] = v;
  }
}

static char serial_line_buf[128] = {0};
static int serial_line_pos = 0;

void process_serial_line(const char *line) {
  char buf[128];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  if (strncmp(buf, "SENS:", 5) == 0) {
    char tmp[128];
    strncpy(tmp, buf + 5, sizeof(tmp) - 1);
    char *tok = strtok(tmp, ":");
    while (tok) {
      char *eq = strchr(tok, '=');
      if (eq) {
        *eq = '\0';
        int k = atoi(tok);
        int v = atoi(eq + 1);
        if (k >= 0 && k < KANDO_COUNT) {
          kando[k] = v;
        }
      }
      tok = strtok(NULL, ":");
    }
    Serial.println("OK:SENS");
  } else if (strcmp(buf, "GETS") == 0) {
    for (int i = 0; i < KANDO_COUNT; i++) {
      Serial.print(i);
      Serial.print(":");
      Serial.println(kando[i]);
    }
    Serial.println("OK:GETS");
  } else if (strcmp(buf, "SAVE") == 0) {
    saveSettings();
    Serial.println("OK:SAVE");
  } else if (strcmp(buf, "RAWON") == 0) {
    raw_debug_mode = true;
    Serial.println("RAW_START");
  } else if (strcmp(buf, "RAWOFF") == 0) {
    raw_debug_mode = false;
    Serial.println("RAW_STOP");
  }
}

void serial_task(void) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_line_pos > 0) {
        serial_line_buf[serial_line_pos] = '\0';
        process_serial_line(serial_line_buf);
        serial_line_pos = 0;
      }
    } else {
      if (serial_line_pos < (int)sizeof(serial_line_buf) - 1) {
        serial_line_buf[serial_line_pos++] = c;
      }
    }
  }

  if (raw_debug_mode) {
    Serial.print("RAW:");
    Serial.print(last_deltas[0]); Serial.print(",");
    Serial.print(last_deltas[1]); Serial.print(",");
    Serial.print(last_deltas[2]); Serial.print(",");
    Serial.println(last_deltas[3]);
    delay(15);
  }
}

void send_keyboard_report(uint16_t s, uint8_t piezo_hits) {
  for (int i = 0; i < 4; i++) {
    if (piezo_hits & (1 << i)) {
      uint8_t rep[8] = {0, 0, key_map[i], 0, 0, 0, 0, 0};
      usb_hid.sendReport(REPORT_ID_KEYBOARD, rep, sizeof(rep));
      delay(kando[7]);
      uint8_t empty[8] = {0};
      usb_hid.sendReport(REPORT_ID_KEYBOARD, empty, sizeof(empty));
    }
  }
}

void loop() {
  if (!switchMode) serial_task(); 

  if (switchMode && TinyUSBDevice.suspended()) {
    if (readSNES() & ((1 << SNES_SELECT) | (1 << SNES_START))) TinyUSBDevice.remoteWakeup();
    delay(10);
    return;
  }

  if (!usb_hid.ready()) { delay(1); return; }

  uint16_t s = readSNES();

  static bool prev_select = false;
  static uint32_t select_hold_start = 0;
  static bool action_triggered = false;
  bool now_select = (s & (1 << SNES_SELECT)) != 0;

  if (now_select && !prev_select) {
    select_hold_start = millis();
    action_triggered = false;
  } else if (now_select && prev_select) {
    if (!action_triggered && !settingMode && !autoCalibMode) {
      if (millis() - select_hold_start >= 1000) {
        autoCalibMode = true;
        calibTargetSensor = 0;
        calibHitCount = 0;
        memset(calibCrosstalkMax, 0, sizeof(calibCrosstalkMax));
        action_triggered = true;
        displaySetting();
        pixels.setPixelColor(0, pixels.Color(255, 200, 0));
        pixels.show();

        Serial.println("=== GOD CALIBRATION STARTED ===");
        Serial.print("Target Sensor [0]: ");
        Serial.println(kandoNames[calibTargetSensor]);
      }
    }
  } else if (!now_select && prev_select) {
    if (!action_triggered && !settingMode && !autoCalibMode) {
      settingMode = true;
      currentSetting = 0;
      action_triggered = true;
      displaySetting();
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));
      pixels.show();
    }
  }
  prev_select = now_select;

  if (settingMode) {
    static bool last_up = false, last_down = false, last_a = false, last_b = false;
    bool up_now   = (s & (1 << SNES_UP)) != 0;
    bool down_now = (s & (1 << SNES_DOWN)) != 0;
    bool a_now    = (s & (1 << SNES_A)) != 0;
    bool b_now    = (s & (1 << SNES_B)) != 0;

    if (up_now && !last_up)     { kando[currentSetting]++; displaySetting(); }
    if (down_now && !last_down) { kando[currentSetting]--; displaySetting(); }
    if (b_now && !last_b) { 
      settingMode = false; 
      pixels.setPixelColor(0, pixels.Color(0, 255, 0)); 
      pixels.show(); 
    }
    if (a_now && !last_a) {
      currentSetting++;
      if (currentSetting >= KANDO_COUNT) {
        currentSetting = 0;
        settingMode = false;
        saveSettings();
        pixels.setPixelColor(0, pixels.Color(0, 255, 0));
        pixels.show();
      } else {
        displaySetting();
      }
    }
    last_up = up_now; last_down = down_now; last_a = a_now; last_b = b_now;
    return; 
  }

  if (autoCalibMode) {
    static bool last_a = false, last_b = false;
    bool a_now = (s & (1 << SNES_A)) != 0;
    bool b_now = (s & (1 << SNES_B)) != 0;

    uint8_t hit_detected = read_piezo_hits();

    if (calibHitCount >= 20 || (a_now && !last_a)) {
      int32_t minVal = 999999, maxVal = 0;
      int validCnt = 0;
      for (int h = 0; h < calibHitCount; h++) {
        if (calibSensorMax[calibTargetSensor][h] > 0) {
          if (calibSensorMax[calibTargetSensor][h] < minVal) minVal = calibSensorMax[calibTargetSensor][h];
          if (calibSensorMax[calibTargetSensor][h] > maxVal) maxVal = calibSensorMax[calibTargetSensor][h];
          validCnt++;
        }
      }
      if (validCnt >= 3) {
        int recommended = minVal + (maxVal - minVal) / 4;
        if (recommended < 15) recommended = 15;
        kando[calibTargetSensor] = recommended; 
      }

      calibTargetSensor++;
      calibHitCount = 0;
      
      if (calibTargetSensor >= 4) {
        int maxCross = 0;
        for (int i = 0; i < 4; i++) {
          if (calibCrosstalkMax[i] > maxCross) maxCross = calibCrosstalkMax[i];
        }
        int calculatedGuard = 25 + (maxCross / 10);
        if (calculatedGuard > 60) calculatedGuard = 60;

        kando[6]  = calculatedGuard; // CROSS_IG
        kando[10] = calculatedGuard; // FACE_LR
        kando[11] = calculatedGuard; // EDGE_LR

        autoCalibMode = false;
        saveSettings();
        pixels.setPixelColor(0, pixels.Color(0, 255, 0));
        pixels.show();

        Serial.println("=== GOD CALIBRATION COMPLETED ===");
        Serial.print("New Thresholds -> L_EDGE:"); Serial.print(kando[0]);
        Serial.print(", L_FACE:"); Serial.print(kando[1]);
        Serial.print(", R_FACE:"); Serial.print(kando[2]);
        Serial.print(", R_EDGE:"); Serial.println(kando[3]);
        Serial.print("Calculated Guard (CROSS/LR): ");
        Serial.println(calculatedGuard);
        Serial.println("Settings saved to EEPROM.");

      } else {
        displaySetting();
        Serial.print("-> Next Target Sensor [");
        Serial.print(calibTargetSensor);
        Serial.print("]: ");
        Serial.println(kandoNames[calibTargetSensor]);
      }
    }
    
    if (b_now && !last_b) {
      autoCalibMode = false;
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
      pixels.show();
    }
    
    last_a = a_now; last_b = b_now;
    return;
  }

  uint8_t piezo_hits = read_piezo_hits();

  static uint8_t piezo_hold = 0;
  static uint32_t piezo_release_time[4] = {0, 0, 0, 0};
  uint32_t now = millis();
  for (int i = 0; i < 4; i++) {
    if (piezo_hits & (1 << i)) {
      piezo_hold |= (1 << i);
      piezo_release_time[i] = now + (uint32_t)kando[7];
    } else if (now > piezo_release_time[i]) {
      piezo_hold &= ~(1 << i);
    }
  }

  if (!switchMode) {
    send_keyboard_report(s, piezo_hits);
    delay(2);
    return;
  }

  pokken_report_t report = {0};

  if (s & (1 << SNES_Y)) report.buttons |= (1 << POKKEN_Y);
  if (s & (1 << SNES_B)) report.buttons |= (1 << POKKEN_B);
  if (s & (1 << SNES_A)) report.buttons |= (1 << POKKEN_A);
  if (s & (1 << SNES_X)) report.buttons |= (1 << POKKEN_X);
  if (s & (1 << SNES_L)) report.buttons |= (1 << POKKEN_L);
  if (s & (1 << SNES_R)) report.buttons |= (1 << POKKEN_R);

  if (s & (1 << SNES_SELECT) && s & (1 << SNES_START)) {
    report.buttons |= (1 << POKKEN_HOME);
  } else {
    if (s & (1 << SNES_SELECT)) report.buttons |= (1 << POKKEN_MINUS);
    if (s & (1 << SNES_START))  report.buttons |= (1 << POKKEN_PLUS);
  }

  if (piezo_hold & (1 << 0)) report.buttons |= (1 << POKKEN_ZL);     
  if (piezo_hold & (1 << 1)) report.buttons |= (1 << POKKEN_LCLICK); 
  if (piezo_hold & (1 << 2)) report.buttons |= (1 << POKKEN_RCLICK); 
  if (piezo_hold & (1 << 3)) report.buttons |= (1 << POKKEN_ZR);     

  report.hat = calcHat(s & (1 << SNES_UP), s & (1 << SNES_DOWN),
                       s & (1 << SNES_LEFT), s & (1 << SNES_RIGHT));

  report.x = 2048; report.y = 2048; report.z = 2048; report.rz = 2048;

  usb_hid.sendReport(0, &report, sizeof(report));
  delay(2);
}
