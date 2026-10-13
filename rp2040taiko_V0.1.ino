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

// RP2040-ZeroのオンボードNeoPixelピン
#define PIN_NEOPIXEL 16
#define NUM_PIXELS   1
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
bool suppress_normal_until_release = false;
int currentSetting = 0;
#define KANDO_COUNT 12
const char* kandoNames[KANDO_COUNT] = {
  "L_EDGE", "L_FACE", "R_FACE", "R_EDGE", "NEXT_B",
  "EDGE_C", "IGNORE_D", "HOLD_MS", "SGL_A", "SGL_AE",
  "FACE_LR", "EDGE_LR"
};
#define EEPROM_SIZE 32

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

int kando[KANDO_COUNT] = {80, 50, 50, 80, 128, 1, 48, 5, 56, 17, 0, 0};
long int result_ac[4] = {0, 0, 0, 0};

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
        p.pair_delay_ms[route] = (uint16_t)kando[6];
      }
    }
  }
  uint8_t r_lr = cal_route_index(1, 2), r_rl = cal_route_index(2, 1); 
  p.pair_delay_ms[r_lr] = (uint16_t)kando[10];
  p.pair_delay_ms[r_rl] = (uint16_t)kando[10];
  uint8_t r_le = cal_route_index(0, 3), r_el = cal_route_index(3, 0); 
  p.pair_delay_ms[r_le] = (uint16_t)kando[11];
  p.pair_delay_ms[r_el] = (uint16_t)kando[11];
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

uint8_t const desc_hid_report[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
  0x09, 0x05,        // Usage (Game Pad)
  0xA1, 0x01,        // Collection (Application)
    0x15, 0x00,      //   Logical Minimum (0)
    0x25, 0x01,      //   Logical Maximum (1)
    0x35, 0x00,      //   Physical Minimum (0)
    0x45, 0x01,      //   Physical Maximum (1)
    0x75, 0x01,      //   Report Size (1)
    0x95, 0x0D,      //   Report Count (13)
    0x05, 0x09,      //   Usage Page (Button)
    0x19, 0x01,      //   Usage Minimum (Button 1)
    0x29, 0x0D,      //   Usage Maximum (Button 13)
    0x81, 0x02,      //   Input (Data,Var,Abs)
    
    0x75, 0x03,      //   Report Size (3)
    0x95, 0x01,      //   Report Count (1)
    0x81, 0x03,      //   Input (Const,Var,Abs)

    0x05, 0x01,      //   Usage Page (Generic Desktop Ctrls)
    0x25, 0x07,      //   Logical Maximum (7)
    0x46, 0x3B, 0x01,//   Physical Maximum (315)
    0x75, 0x04,      //   Report Size (4)
    0x95, 0x01,      //   Report Count (1)
    0x65, 0x14,      //   Unit (System: English Rotation, Length: Centimeter)
    0x09, 0x39,      //   Usage (Hat switch)
    0x81, 0x42,      //   Input (Data,Var,Abs,Null)

    0x75, 0x04,      //   Report Size (4)
    0x95, 0x01,      //   Report Count (1)
    0x81, 0x03,      //   Input (Const,Var,Abs)

    0x15, 0x00,      //   Logical Minimum (0)
    0x26, 0xFF, 0x0F,//   Logical Maximum (4095)
    0x75, 0x10,      //   Report Size (16)
    0x95, 0x04,      //   Report Count (4)
    0x09, 0x30,      //   Usage (X)
    0x09, 0x31,      //   Usage (Y)
    0x09, 0x32,      //   Usage (Z)
    0x09, 0x35,      //   Usage (Rz)
    0x81, 0x02,      //   Input (Data,Var,Abs)
  0xC0               // End Collection
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

#define POKKEN_Y       0 
#define POKKEN_B       1 
#define POKKEN_A       2 
#define POKKEN_X       3 
#define POKKEN_L       4 
#define POKKEN_R       5 
#define POKKEN_ZL      6 
#define POKKEN_ZR      7 
#define POKKEN_MINUS   8 
#define POKKEN_PLUS    9 
#define POKKEN_LCLICK  10 
#define POKKEN_RCLICK  11 
#define POKKEN_HOME    12 

uint16_t readSNES();
void loadSettings();
void saveSettings();

void setup() {
  // NeoPixel初期化と点灯準備
  pixels.begin();
  pixels.setBrightness(30); // 眩しすぎないように輝度30 (0~255)
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
    
    // Switchモード時は青色 (Blue)
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.show();
  } else {
    USBDevice.setID(0xCAFE, 0x4004);
    usb_hid.setReportDescriptor(desc_hid_report_keyboard, sizeof(desc_hid_report_keyboard));
    
    // PCモード時は緑色 (Green)
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();
  }

  usb_hid.setPollInterval(2);
  usb_hid.begin();

  if (switchMode) {
    while (!USBDevice.mounted()) {
      delay(10);
    }
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

  if (!switchMode) {
    Serial.println("=== 起動時にEEPROMから読み込んだ設定値 ===");
    for (int i = 0; i < KANDO_COUNT; i++) {
      Serial.print(kandoNames[i]);
      Serial.print(" = ");
      Serial.println(kando[i]);
    }
    Serial.println("==========================================");
  }

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
    if (digitalRead(PIN_DATA) == LOW) {
      state |= (1 << i);
    }
    digitalWrite(PIN_CLOCK, LOW);
    delayMicroseconds(6);
    digitalWrite(PIN_CLOCK, HIGH);
    delayMicroseconds(6);
  }
  return state;
}

uint8_t calcHat(bool up, bool down, bool left, bool right) {
  if (up    && right) return 1;
  if (down  && right) return 3;
  if (down  && left)  return 5;
  if (up    && left)  return 7;
  if (up)    return 0;
  if (right) return 2;
  if (down)  return 4;
  if (left)  return 6;
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
    adc_select_input(i); 
    long int result = adc_read();
    int32_t delta = (int32_t)(result - result_ac[i]);

    if (piezo_gate.accept((uint8_t)i, delta, main_timer, piezo_profile)) {
      hits |= (1 << i);
    }
    result_ac[i] = result;
  }
  return hits;
}

void displayMode() {
  if (!oled_ok) return;
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println(settingMode ? "SET MODE" : (switchMode ? "SW MODE" : "PC MODE"));
  display.display();
}

void displaySetting() {
  if (!oled_ok) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SETTING (Up/Down:+- A:Next B:Cancel)");
  display.setTextSize(2);
  display.setCursor(0, 24);
  display.print(kandoNames[currentSetting]);
  display.print(":");
  display.println(kando[currentSetting]);
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
  if (strncmp(line, "SENS:", 5) == 0) {
    char tmp[128];
    strncpy(tmp, line + 5, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';
    char *tok = strtok(tmp, ":");
    while (tok) {
      char *eq = strchr(tok, '=');
      if (eq) {
        *eq = '\0';
        int k = atoi(tok), v = atoi(eq + 1);
        if (k >= 0 && k < KANDO_COUNT) kando[k] = v;
      }
      tok = strtok(NULL, ":");
    }
    Serial.println("OK");
  } else if (strncmp(line, "KEY:", 4) == 0) {
    char tmp[32];
    strncpy(tmp, line + 4, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';
    char *tok = strtok(tmp, ":");
    while (tok) {
      char *eq = strchr(tok, '=');
      if (eq) {
        *eq = '\0';
        int k = atoi(tok), v = atoi(eq + 1);
        if (k >= 0 && k < 4) key_map[k] = (uint8_t)v;
      }
      tok = strtok(NULL, ":");
    }
    Serial.println("OK");
  } else if (strcmp(line, "GETS") == 0) {
    for (int i = 0; i < KANDO_COUNT; i++) { Serial.print(i); Serial.print(":"); Serial.println(kando[i]); }
    Serial.println("OK");
  } else if (strcmp(line, "GETK") == 0) {
    for (int i = 0; i < 4; i++) { Serial.print(i); Serial.print(":"); Serial.println(key_map[i]); }
    Serial.println("OK");
  } else if (strcmp(line, "SAVE") == 0) {
    saveSettings();
    Serial.println("OK");
  } else if (strcmp(line, "LOAD") == 0) {
    loadSettings();
    Serial.println("OK");
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
    } else if (serial_line_pos < (int)sizeof(serial_line_buf) - 1) {
      serial_line_buf[serial_line_pos++] = c;
    }
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

  bool select_now = s & (1 << SNES_SELECT);
  static bool last_select = false;
  if (select_now && !last_select && !settingMode) {
    settingMode = true;
    currentSetting = 0;
    displaySetting();
    
    // ★設定モード突入時はLEDを赤色 (Red) に変更
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();

    Serial.println("=== 設定モード突入 ===");
    Serial.print(kandoNames[currentSetting]); Serial.print(" = "); Serial.println(kando[currentSetting]);
    { uint8_t rep[8]={0,0,HID_KEY_S,0,0,0,0,0}, e[8]={0}; 
      usb_hid.sendReport(REPORT_ID_KEYBOARD, rep, sizeof(rep)); delay(30);
      usb_hid.sendReport(REPORT_ID_KEYBOARD, e, sizeof(e)); }
  }
  last_select = select_now;

  if (settingMode) {
    static bool last_up = false, last_down = false, last_a = false, last_b = false;
    bool up_now   = s & (1 << SNES_UP);
    bool down_now = s & (1 << SNES_DOWN);
    bool a_now    = s & (1 << SNES_A);
    bool b_now    = s & (1 << SNES_B);

    if (up_now && !last_up) {
      kando[currentSetting]++; displaySetting();
      Serial.print(kandoNames[currentSetting]); Serial.print(" -> "); Serial.println(kando[currentSetting]);
    }
    if (down_now && !last_down) {
      kando[currentSetting]--; displaySetting();
      Serial.print(kandoNames[currentSetting]); Serial.print(" -> "); Serial.println(kando[currentSetting]);
    }
    if (b_now && !last_b) {
      settingMode = false; displayMode();
      
      // キャンセル時はPCモードの緑色 (Green) に戻す
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
      pixels.show();

      suppress_normal_until_release = true;
      Serial.println("設定モード キャンセル");
      uint8_t rep[8]={0,0,HID_KEY_C,0,0,0,0,0}, e[8]={0}; 
      usb_hid.sendReport(REPORT_ID_KEYBOARD, rep, sizeof(rep)); delay(30);
      usb_hid.sendReport(REPORT_ID_KEYBOARD, e, sizeof(e));
    }
    if (a_now && !last_a) {
      Serial.print(kandoNames[currentSetting]); Serial.print(" 確定 = "); Serial.println(kando[currentSetting]);
      currentSetting++;
      if (currentSetting >= KANDO_COUNT) {
        currentSetting = 0;
        settingMode = false;
        
        // 設定完了時もPCモードの緑色 (Green) に戻す
        pixels.setPixelColor(0, pixels.Color(0, 255, 0));
        pixels.show();

        suppress_normal_until_release = true;
        saveSettings();
        displayMode();
        Serial.println("=== EEPROMへ保存しました ===");
        for (int i = 0; i < KANDO_COUNT; i++) {
          Serial.print(kandoNames[i]); Serial.print(" = "); Serial.println(kando[i]);
        }
        uint8_t rep[8]={0,0,HID_KEY_E,0,0,0,0,0}, e[8]={0}; 
        usb_hid.sendReport(REPORT_ID_KEYBOARD, rep, sizeof(rep)); delay(30);
        usb_hid.sendReport(REPORT_ID_KEYBOARD, e, sizeof(e));
      } else {
        displaySetting();
        Serial.print("次: "); Serial.print(kandoNames[currentSetting]); Serial.print(" = "); Serial.println(kando[currentSetting]);
      }
    }
    last_up = up_now; last_down = down_now; last_a = a_now; last_b = b_now;
    return; 
  }

  if (suppress_normal_until_release) {
    if (!(s & (1 << SNES_A)) && !(s & (1 << SNES_B))) {
      suppress_normal_until_release = false;
    } else {
      return; 
    }
  }
  static uint8_t last_codes[2] = {0, 0};
  uint8_t codes[2] = {0, 0};
  if (s & (1 << SNES_UP))    codes[0] = HID_KEY_U; 
  if (s & (1 << SNES_DOWN))  codes[0] = HID_KEY_N; 
  if (s & (1 << SNES_LEFT))  codes[0] = HID_KEY_L; 
  if (s & (1 << SNES_RIGHT)) codes[0] = HID_KEY_T; 
  if (s & (1 << SNES_A))      codes[1] = HID_KEY_1; 
  else if (s & (1 << SNES_B)) codes[1] = HID_KEY_2; 
  else if (s & (1 << SNES_X)) codes[1] = HID_KEY_3; 
  else if (s & (1 << SNES_Y)) codes[1] = HID_KEY_4; 
  else if (s & (1 << SNES_L)) codes[1] = HID_KEY_5; 
  else if (s & (1 << SNES_R)) codes[1] = HID_KEY_6; 

  if (codes[0] != last_codes[0] || codes[1] != last_codes[1]) {
    last_codes[0] = codes[0];
    last_codes[1] = codes[1];
    uint8_t rep[8] = {0, 0, codes[0], codes[1], 0, 0, 0, 0};
    usb_hid.sendReport(REPORT_ID_KEYBOARD, rep, sizeof(rep));
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

  static uint8_t last_shown = 0xFF;
  if (switchMode && oled_ok && piezo_hold != last_shown) {
    last_shown = piezo_hold;
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("SW MODE");
    display.setCursor(0, 24);
    display.print("L:"); display.print((piezo_hold & 0x01) ? "E" : "-");
    display.print((piezo_hold & 0x02) ? "F" : "-");
    display.print(" R:"); display.print((piezo_hold & 0x04) ? "F" : "-");
    display.println((piezo_hold & 0x08) ? "E" : "-");
    display.display();
  }

  if (!switchMode) {
    send_keyboard_report(s, piezo_hits);
    delay(4);
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

  if (piezo_hold & (1 << 0)) report.buttons |= (1 << POKKEN_LCLICK);
  if (piezo_hold & (1 << 1)) report.buttons |= (1 << POKKEN_ZL);
  if (piezo_hold & (1 << 2)) report.buttons |= (1 << POKKEN_ZR);
  if (piezo_hold & (1 << 3)) report.buttons |= (1 << POKKEN_RCLICK);

  report.hat = calcHat(s & (1 << SNES_UP), s & (1 << SNES_DOWN),
                        s & (1 << SNES_LEFT), s & (1 << SNES_RIGHT));

  report.x = 2048;
  report.y = 2048;
  report.z = 2048;
  report.rz = 2048;

  usb_hid.sendReport(0, &report, sizeof(report));
  delay(8);
}
