/*
 * Taiko Controller Ultimate Firmware for Waveshare RP2040-Zero (v0.9)
 * - Multi-Mode USB HID (Switch Tatacon / PC Keyboard)
 * - 4-Channel Piezo with Smart Noise Gate, EMA Filter & Pair Delay
 * - SNES Controller UI & EEPROM Parameter Persistence
 *
 * v0.9 変更点
 *  - Switchモードではシリアルを完全に切る(Serial.end())。シリアルが立ち上がるとSwitchに認識されない
 *  - PCモードのSelect設定メニューを、OLEDとシリアルモニターの両方に表示するよう復活
 *  - Web用に打撃イベント "HIT:ch" を送信(PCモードのみ)。Webの打撃モニター/カウンターが反応する
 *  - RAW波形は delay() ではなく一定間隔(40ms)+区間ピーク保持で送信(送りすぎによる詰まりを防止)
 *  - 設定値は0〜255に制限(EEPROMは1バイト保存のため、範囲外は化けていた)
 *  - 設定モード中も圧電のベースラインを更新(設定モード終了直後の誤検出を防止)
 *  - 設定モードはPCモードのみ(Switchモードでは Select は「-」ボタン)。ALLOW_SETTING_IN_SWITCH で変更可
 *  - 自動キャリブレーションは廃止
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

// USB Vendor/Product for Switch Tatacon / Hori
#define SWITCH_VID  0x0F0D
#define SWITCH_PID  0x0092

enum UsbMode {
  MODE_SWITCH_TATACON = 0,
  MODE_KEYBOARD_P1,
  MODE_DEBUG
};
UsbMode currentUsbMode = MODE_SWITCH_TATACON;

// TinyUSB HID Report Descriptors
enum { REPORT_ID_KEYBOARD = 1, REPORT_ID_JOYSTICK = 0 };

uint8_t const desc_hid_report_joystick[] = {
  0x05, 0x01,      0x09, 0x05,      0xA1, 0x01,      // Generic Desktop -> Gamepad
    0x15, 0x00,      0x25, 0x01,      0x35, 0x00,      0x45, 0x01,
    0x75, 0x01,      0x95, 0x0E,      0x05, 0x09,      0x19, 0x01,
    0x29, 0x0E,      0x81, 0x02,      0x75, 0x02,      0x95, 0x01,
    0x81, 0x03,      0x05, 0x01,      0x25, 0x07,      0x46, 0x3B, 0x01,
    0x75, 0x04,      0x95, 0x01,      0x65, 0x14,      0x09, 0x39,
    0x81, 0x42,      0x75, 0x04,      0x95, 0x01,      0x81, 0x03,
    0x15, 0x00,      0x26, 0xFF, 0x0F,0x75, 0x10,      0x95, 0x04,
    0x09, 0x30,      0x09, 0x31,      0x09, 0x32,      0x09, 0x35,
    0x81, 0x02,      0xC0
};

uint8_t const desc_hid_report_keyboard[] = {
  TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD))
};

Adafruit_USBD_HID usb_hid;

// 12項目のパラメータ定義 (論理順: D, F, J, K)
#define KANDO_COUNT 12
int kando[KANDO_COUNT] = {
    80,  // [0] D_L-EDGE (左カッ)
    160, // [1] F_L-FACE (左ドン)
    160, // [2] J_R-FACE (右ドン)
    80,  // [3] K_R-EDGE (右カッ)
    30,  // [4] FACE_INTV
    12,  // [5] EDGE_INTV
    35,  // [6] CROSS_IG
    30,  // [7] HOLD_MS
    0,   // [8] FACE_SGL
    0,   // [9] EDGE_SGL
    40,  // [10] FACE_LR
    40   // [11] EDGE_LR
};

const char *kandoNames[KANDO_COUNT] = {
    "D_L-EDGE", "F_L-FACE", "J_R-FACE", "K_R-EDGE", "FACE_INTV",
    "EDGE_INTV", "CROSS_IG", "HOLD_MS", "FACE_SGL", "EDGE_SGL",
    "FACE_LR", "EDGE_LR"
};
#define EEPROM_SIZE 16

#define PIN_LATCH 0
#define PIN_CLOCK 1
#define PIN_DATA  2

// SNES Button bits
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

bool oled_ok = false;
bool settingMode = false;
int currentSetting = 0;
bool raw_debug_mode = false;
bool serial_on = false;               // PCモードのときだけtrue。Switchモードでは常にfalse
uint16_t raw_peak[4] = {0};           // 前回RAW送信以降のピーク(delta)
uint32_t last_raw_send_ms = 0;

// 1にするとSwitchモードでもSelectで設定モードに入れる(既定0: Selectは「-」ボタンとして使う)
#define ALLOW_SETTING_IN_SWITCH 0

// キャリブレーション・プロファイル構造体
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
      } else if (s != t) {
        bool is_face = (s == 1 || s == 2);
        p.pair_delay_ms[route] = (uint16_t)(is_face ? kando[10] : kando[11]);
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
    if (now_ms - (face ? last_face_ms_ : last_edge_ms_) < (face ? p.self_face_b : p.self_edge_c)) return false;
    if (now_ms - last_hit_ms_[target] <= (face ? p.self_face_a : p.self_edge_a)) return false;

    for (uint8_t source = 0; source < 4; source++) {
      if (source == target || (seen_mask_ & (1u << source)) == 0) continue;
      uint8_t route = cal_route_index(source, target);
      uint32_t elapsed = now_ms - last_hit_ms_[source];
      if (cal_same_type(source, target)) {
        if (elapsed < p.pair_delay_ms[route]) return false;
      } else if (elapsed <= p.pair_delay_ms[route]) {
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

uint8_t getHidHat(uint16_t s) {
  bool up = s & (1 << SNES_UP);
  bool down = s & (1 << SNES_DOWN);
  bool left = s & (1 << SNES_LEFT);
  bool right = s & (1 << SNES_RIGHT);
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

static float smooth_val[4] = {0};

uint8_t read_piezo_hits() {
  static uint64_t last_sample_us = 0;
  uint64_t now_us = to_us_since_boot(get_absolute_time());
  if (now_us - last_sample_us < 10) return 0;
  last_sample_us = now_us;

  build_profile_from_kando(piezo_profile); 
  uint32_t main_timer = (uint32_t)millis();
  uint8_t hits = 0;
  const int32_t SMART_GATE = 18; // スマートノイズゲート

  for (int physical_i = 0; physical_i < 4; physical_i++) {
    // 物理ピンの並びを反転マッピング (physical 0 -> logical 3, physical 3 -> logical 0)
    int logical_i = 3 - physical_i;

    adc_select_input(3 - physical_i); 
    float raw_adc = (float)adc_read();

    if (smooth_val[logical_i] == 0) smooth_val[logical_i] = raw_adc;
    smooth_val[logical_i] = (smooth_val[logical_i] * 0.4f) + (raw_adc * 0.6f);

    int32_t raw_delta = (int32_t)(smooth_val[logical_i] - (float)result_ac[logical_i]);
    result_ac[logical_i] = (long int)smooth_val[logical_i];

    int32_t delta = (raw_delta > 0) ? raw_delta : -raw_delta;
    
    if (delta <= SMART_GATE) {
      delta = 0;
    } else {
      delta -= SMART_GATE;
    }

    last_deltas[logical_i] = delta;
    if (delta > (long int)delta_max[logical_i]) delta_max[logical_i] = (uint16_t)delta;
    if (delta > (int32_t)raw_peak[logical_i]) raw_peak[logical_i] = (uint16_t)(delta > 65535 ? 65535 : delta);

    if (!settingMode) {
      if (piezo_gate.accept((uint8_t)logical_i, delta, main_timer, piezo_profile)) {
        hits |= (1 << logical_i);
      }
    }
  }
  return hits;
}

void loadSettings() {
  for (int i = 0; i < KANDO_COUNT; i++) {
    uint8_t v = EEPROM.read(i);
    if (v != 0xFF) kando[i] = v;
  }
}

void saveSettings() {
  for (int i = 0; i < KANDO_COUNT; i++) EEPROM.write(i, (uint8_t)kando[i]);
  EEPROM.commit();
}

// ---------------- OLED表示 (128x64) ----------------
// 文字を画面中央に寄せて表示する。size=2は1文字12px(最大10文字)、size=1は6px(最大21文字)
void drawCentered(const char *txt, int y, int size) {
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(size);
  display.getTextBounds(txt, 0, y, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - (int)w) / 2, y);
  display.print(txt);
}

// 通常画面(起動時と、設定モードから戻ったとき)
void displayMode() {
  if (!oled_ok) return;
  bool sw = (currentUsbMode == MODE_SWITCH_TATACON);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawCentered(sw ? "SW MODE" : "PC MODE", 12, 2);
  drawCentered(sw ? "Switch Tatacon" : "Keyboard (D F J K)", 38, 1);
  if (!sw) drawCentered("SELECT: SETTING", 54, 1);
  display.display();
}

// 設定画面: 1行目に項目番号、2行目に項目名、3行目に値、最下段に0$301C255のバー
void displaySetting() {
  if (!oled_ok) return;
  char title[24];
  snprintf(title, sizeof(title), "SETTING %d/%d", currentSetting + 1, KANDO_COUNT);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(title);
  drawCentered(kandoNames[currentSetting], 11, 2);
  char val[8];
  snprintf(val, sizeof(val), "%d", kando[currentSetting]);
  drawCentered(val, 31, 3);
  display.drawRect(0, 57, SCREEN_WIDTH, 7, SSD1306_WHITE);
  int fill = (kando[currentSetting] * (SCREEN_WIDTH - 4)) / 255;
  if (fill > 0) display.fillRect(2, 59, fill, 3, SSD1306_WHITE);
  display.display();
}

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
  uint16_t s = readSNES();
  if (s & (1 << SNES_START)) {
    currentUsbMode = MODE_KEYBOARD_P1;
  } else {
    currentUsbMode = MODE_SWITCH_TATACON;
  }

  if (currentUsbMode == MODE_SWITCH_TATACON) {
    Serial.end(); // Switchモードはシリアルを完全に切る(CDCが混ざるとSwitchに認識されない)
    serial_on = false;
    USBDevice.setID(SWITCH_VID, SWITCH_PID);
    USBDevice.setManufacturerDescriptor("HORI CO., LTD.");
    USBDevice.setProductDescriptor("HORIPAD");
    usb_hid.setReportDescriptor(desc_hid_report_joystick, sizeof(desc_hid_report_joystick));
    pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // 青: Switchモード
  } else {
    USBDevice.setID(0xCAFE, 0x4004);
    usb_hid.setReportDescriptor(desc_hid_report_keyboard, sizeof(desc_hid_report_keyboard));
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // 緑: キーボードモード
  }
  pixels.show();

  usb_hid.setPollInterval(2);
  usb_hid.begin();

  if (currentUsbMode == MODE_SWITCH_TATACON) {
    while (!USBDevice.mounted()) { delay(10); }
    delay(500);
  } else {
    Serial.begin(115200); // PCモードだけシリアルを有効化(USB初期化の後)
    serial_on = true;
    delay(500);
  }

  adc_init();
  for (int pin = 26; pin <= 29; pin++) adc_gpio_init(pin);

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
    displayMode();
  }
}

static char serial_line_buf[128] = {0};
static int serial_line_pos = 0;

void process_serial_line(const char *line) {
  if (strncmp(line, "SENS:", 5) == 0) {
    char tmp[128];
    strncpy(tmp, line + 5, sizeof(tmp) - 1);
    char *tok = strtok(tmp, ":");
    while (tok) {
      char *eq = strchr(tok, '=');
      if (eq) {
        *eq = '\0';
        int k = atoi(tok);
        int v = atoi(eq + 1);
        if (k >= 0 && k < KANDO_COUNT) kando[k] = constrain(v, 0, 255);
      }
      tok = strtok(NULL, ":");
    }
    Serial.println("OK:SENS");
  } else if (strcmp(line, "GETS") == 0) {
    Serial.println("--- CURRENT KANDO PARAMETERS ---");
    for (int i = 0; i < KANDO_COUNT; i++) {
      Serial.print(i); Serial.print(" ["); Serial.print(kandoNames[i]); Serial.print("]: ");
      Serial.println(kando[i]);
    }
    Serial.println("OK:GETS");
  } else if (strcmp(line, "SAVE") == 0) {
    saveSettings();
    Serial.println("OK:SAVE");
  } else if (strcmp(line, "RAWON") == 0) {
    raw_debug_mode = true;
    Serial.println("RAW_START");
  } else if (strcmp(line, "RAWOFF") == 0) {
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

  // RAW波形: delay()で止めず、40msごとに「その区間のピーク値」を送る(叩いた瞬間を取りこぼさない)
  if (raw_debug_mode && (millis() - last_raw_send_ms >= 40)) {
    last_raw_send_ms = millis();
    Serial.print("RAW:");
    Serial.print(raw_peak[0]); Serial.print(",");
    Serial.print(raw_peak[1]); Serial.print(",");
    Serial.print(raw_peak[2]); Serial.print(",");
    Serial.println(raw_peak[3]);
    for (int i = 0; i < 4; i++) raw_peak[i] = 0;
  }
}

// 設定モードのシリアル表示用ヘルパー(PCモードのときだけ出力)
void logSetting(const char *prefix) {
  if (!serial_on) return;
  Serial.print(prefix);
  Serial.print(currentSetting);
  Serial.print(" [");
  Serial.print(kandoNames[currentSetting]);
  Serial.print("]: ");
  Serial.println(kando[currentSetting]);
}

void setLedNormal() {
  pixels.setPixelColor(0, currentUsbMode == MODE_SWITCH_TATACON ? pixels.Color(0, 0, 255) : pixels.Color(0, 255, 0));
  pixels.show();
  displayMode(); // 設定モードから戻ったとき、OLEDも通常画面に戻す
}

void loop() {
  if (serial_on) serial_task();

  if (!usb_hid.ready()) { delay(1); return; }

  uint16_t s = readSNES();

  // 設定モード中もベースラインを更新する(判定自体は設定モード中は無効)
  uint8_t piezo_hits = read_piezo_hits();

  // 設定モードに入れるのはPCモードのみ(既定)。Switchモードでは Select は「-」ボタン
  bool setting_allowed = (currentUsbMode == MODE_KEYBOARD_P1) || ALLOW_SETTING_IN_SWITCH;

  static bool prev_select = false;
  bool now_select = (s & (1 << SNES_SELECT)) != 0;

  if (setting_allowed && now_select && !prev_select) {
    settingMode = !settingMode;
    if (settingMode) {
      currentSetting = 0;
      displaySetting();
      pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // 赤: 設定モード
      pixels.show();
      if (serial_on) Serial.println("--- MANUAL SETTING MODE ENTERED ---");
      logSetting("[SETTING] Current -> Index ");
    } else {
      saveSettings();
      setLedNormal();
      if (serial_on) Serial.println("[SETTING] Saved & Exited to Normal Mode.");
    }
  }
  prev_select = now_select;

  if (settingMode) {
    static bool last_up = false, last_down = false, last_a = false, last_b = false;
    bool up_now   = (s & (1 << SNES_UP)) != 0;
    bool down_now = (s & (1 << SNES_DOWN)) != 0;
    bool a_now    = (s & (1 << SNES_A)) != 0;
    bool b_now    = (s & (1 << SNES_B)) != 0;

    if (up_now && !last_up) {
      if (kando[currentSetting] < 255) kando[currentSetting]++;   // EEPROMは1バイト保存なので0〜255
      displaySetting();
      logSetting("[SETTING] UP   Index ");
    }
    if (down_now && !last_down) {
      if (kando[currentSetting] > 0) kando[currentSetting]--;
      displaySetting();
      logSetting("[SETTING] DOWN Index ");
    }
    if (a_now && !last_a) {
      currentSetting++;
      if (currentSetting >= KANDO_COUNT) {
        currentSetting = 0;
        settingMode = false;
        saveSettings();
        setLedNormal();
        if (serial_on) Serial.println("[SETTING] Completed & Saved to EEPROM.");
      } else {
        displaySetting();
        logSetting("[SETTING] Next -> Index ");
      }
    }
    if (b_now && !last_b) {
      settingMode = false;
      saveSettings();
      setLedNormal();
      if (serial_on) Serial.println("[SETTING] Saved & Exited via B Button.");
    }
    last_up = up_now; last_down = down_now; last_a = a_now; last_b = b_now;
    return;
  }

  // 打撃イベントをWebモニターへ通知 (PCモードのみ)
  if (serial_on && piezo_hits) {
    for (int i = 0; i < 4; i++) {
      if (piezo_hits & (1 << i)) { Serial.print("HIT:"); Serial.println(i); }
    }
  }

  static uint8_t piezo_hold = 0;
  static uint32_t piezo_release_time[4] = {0};
  uint32_t now = millis();
  for (int i = 0; i < 4; i++) {
    if (piezo_hits & (1 << i)) {
      piezo_hold |= (1 << i);
      piezo_release_time[i] = now + (uint32_t)kando[7];
    } else if (now > piezo_release_time[i]) {
      piezo_hold &= ~(1 << i);
    }
  }

  // キーボードモードの場合 (D, F, J, K)
  if (currentUsbMode == MODE_KEYBOARD_P1) {
    uint8_t key_map[4] = {HID_KEY_D, HID_KEY_F, HID_KEY_J, HID_KEY_K};
    for (int i = 0; i < 4; i++) {
      if (piezo_hits & (1 << i)) {
        uint8_t rep[8] = {0, 0, key_map[i], 0, 0, 0, 0, 0};
        usb_hid.sendReport(REPORT_ID_KEYBOARD, rep, sizeof(rep));
        delay(kando[7]);
        uint8_t empty[8] = {0};
        usb_hid.sendReport(REPORT_ID_KEYBOARD, empty, sizeof(empty));
      }
    }
    delay(2);
    return;
  }

  // Switch タタコンモード構造体
  typedef struct __attribute__((packed)) {
    uint16_t buttons;
    uint8_t  hat;
    uint16_t x, y, z, rz;
  } switch_tatacon_report_t;

  switch_tatacon_report_t report = {0};

  if (s & (1 << SNES_Y)) report.buttons |= (1 << 0); // Y
  if (s & (1 << SNES_B)) report.buttons |= (1 << 1); // B
  if (s & (1 << SNES_A)) report.buttons |= (1 << 2); // A
  if (s & (1 << SNES_X)) report.buttons |= (1 << 3); // X
  if (s & (1 << SNES_L)) report.buttons |= (1 << 4); // L
  if (s & (1 << SNES_R)) report.buttons |= (1 << 5); // R

  // 縁(Ka) -> ZL / ZR (index 0:左カ=D, index 3:右カ=K)
  if (piezo_hold & (1 << 0)) report.buttons |= (1 << 6); // ZL (左カッ: D)
  if (piezo_hold & (1 << 3)) report.buttons |= (1 << 7); // ZR (右カッ: K)

  if (s & (1 << SNES_SELECT)) report.buttons |= (1 << 8); // -
  if (s & (1 << SNES_START))  report.buttons |= (1 << 9); // +

  // 面(Don) -> L3 / R3 (index 1:左ドン=F, index 2:右ドン=J)
  if (piezo_hold & (1 << 1)) report.buttons |= (1 << 10); // L3 (左ドン: F)
  if (piezo_hold & (1 << 2)) report.buttons |= (1 << 11); // R3 (右ドン: J)

  if (s & (1 << SNES_SELECT) && s & (1 << SNES_START)) {
    report.buttons |= (1 << 12); // HOME
  }

  report.hat = getHidHat(s);
  report.x = 2048; report.y = 2048; report.z = 2048; report.rz = 2048;

  usb_hid.sendReport(REPORT_ID_JOYSTICK, &report, sizeof(report));
  delay(2);
}
