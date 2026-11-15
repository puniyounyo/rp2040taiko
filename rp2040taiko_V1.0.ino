/*
 * Taiko Controller Firmware for Waveshare RP2040-Zero (v1.0)
 * - Multi-Mode USB HID (Switch Tatacon / PC Keyboard)
 * - 4-Channel Piezo with Positive-Delta Detection, Ratio Gate & Pair Delay
 * - SNES Controller UI & EEPROM Parameter Persistence (PC / Switch profiles)
 *
 * Frozen USB identity (do not change without a reason):
 *  - Switch mode: VID 0x0F0D / PID 0x0092, "HORI CO., LTD." / "HORIPAD",
 *    HID report descriptor is byte-identical to v0.9
 *  - PC mode:     VID 0xCAFE / PID 0x4004, keyboard report (D / F / J / K)
 * Mapping (Switch): face(L/R) -> L3 / R3, edge(L/R) -> ZL / ZR,
 *  SNES Select -> -, Start -> +, Select+Start -> HOME
 *
 * v1.0 changes from v0.9
 *  - Detection reverted to the original HIDtaiko method: raw ADC,
 *    positive delta only (no EMA smoothing, no absolute value).
 *    The negative ringing swing of a piezo is no longer counted as a hit,
 *    which was the prime suspect for false detections on a wooden body.
 *  - Ratio gate (winner-take-all): hits are collected for JUDGE_WIN ms,
 *    then only channels whose peak is at least CROSS_RATIO % of the
 *    strongest peak survive. Crosstalk is always weaker than the direct
 *    hit, so this absorbs body-material differences that a fixed
 *    threshold cannot. JUDGE_WIN=0 disables the window (v0.9 behaviour).
 *  - PC and Switch sensitivity profiles are stored separately in EEPROM
 *    (layout v1, auto-migrated from the v0.9 layout). Serial command
 *    COPY2SW copies the PC profile to the Switch profile.
 *  - Setting mode can be entered in Switch mode too: hold SNES Select
 *    for 1 second (a short press is still the "-" button). Serial stays
 *    off in Switch mode, so settings are edited on the OLED only.
 *  - PC keyboard mode now uses the same non-blocking HOLD_MS handling
 *    as Switch mode (v0.9 used a blocking delay, so the same HOLD_MS
 *    felt different between modes).
 *  - Parameters expanded 12 -> 14: [12] CROSS_RATIO, [13] JUDGE_WIN.
 *    Every parameter has its own maximum (EEPROM is still 1 byte each).
 *
 * Hardware: piezo ADC GP26-29, SNES pad GP0/1/2 (LATCH/CLOCK/DATA),
 *  OLED SSD1306 I2C GP4/GP5, NeoPixel GP16.
 * Status: v1.0 is UNVERIFIED on real hardware. Test order is in the
 *  project notes (PC mode first, then Switch cold boot / replug).
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

// TinyUSB HID Report Descriptors (unchanged from v0.9)
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

// 14 parameters (logical order: D, F, J, K + timing + ratio gate)
#define KANDO_COUNT 14
const int kandoDefault[KANDO_COUNT] = {
    80,  // [0] D_L-EDGE (left Ka)
    160, // [1] F_L-FACE (left Don)
    160, // [2] J_R-FACE (right Don)
    80,  // [3] K_R-EDGE (right Ka)
    30,  // [4] FACE_INTV
    12,  // [5] EDGE_INTV
    35,  // [6] CROSS_IG
    30,  // [7] HOLD_MS
    0,   // [8] FACE_SGL
    0,   // [9] EDGE_SGL
    40,  // [10] FACE_LR
    40,  // [11] EDGE_LR
    60,  // [12] CROSS_RATIO (% of strongest peak a crosstalk candidate must reach)
    6    // [13] JUDGE_WIN (ms, hit collection window; 0 = immediate, no ratio gate)
};

const int kandoMax[KANDO_COUNT] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    100, // CROSS_RATIO is a percent
    30   // JUDGE_WIN in ms
};

const char *kandoNames[KANDO_COUNT] = {
    "D_L-EDGE", "F_L-FACE", "J_R-FACE", "K_R-EDGE", "FACE_INTV",
    "EDGE_INTV", "CROSS_IG", "HOLD_MS", "FACE_SGL", "EDGE_SGL",
    "FACE_LR", "EDGE_LR", "CROSS_RATIO", "JUDGE_WIN"
};

// Short names for the OLED setting screen (size-2 text fits 10 chars)
const char *kandoNamesShort[KANDO_COUNT] = {
    "D_L-EDGE", "F_L-FACE", "J_R-FACE", "K_R-EDGE", "FACE_INTV",
    "EDGE_INTV", "CROSS_IG", "HOLD_MS", "FACE_SGL", "EDGE_SGL",
    "FACE_LR", "EDGE_LR", "X-RATIO", "JUDGE_WIN"
};

// Sensitivity profiles: PC and Switch are stored separately
#define PROFILE_PC 0
#define PROFILE_SW 1
int profiles[2][KANDO_COUNT];
int kando[KANDO_COUNT];            // working copy of the active profile
int activeProfile = PROFILE_SW;

// EEPROM layout v1 (64 bytes):
//   0..13  PC profile, 16..29 Switch profile, 32 magic, 33 layout version
// v0.9 used a 16-byte layout with a single profile at 0..11; if the magic
// is missing, those 12 values are migrated into both profiles.
#define EEPROM_SIZE 64
#define EE_OFF_PC 0
#define EE_OFF_SW 16
#define EE_MAGIC_ADDR 32
#define EE_MAGIC 0xA5
#define EE_LAYOUT_VER 1

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

long int prev_raw[4] = {0, 0, 0, 0};
int32_t last_deltas[4] = {0, 0, 0, 0};
uint16_t delta_max[4] = {0};

bool oled_ok = false;
bool settingMode = false;
int currentSetting = 0;
bool raw_debug_mode = false;
bool serial_on = false;               // true only in PC mode; always false in Switch mode
uint16_t raw_peak[4] = {0};           // peak delta since the last RAW send
uint32_t last_raw_send_ms = 0;

// Calibration profile derived from kando (same structure as v0.9)
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

// ---- Piezo detection -------------------------------------------------
// Raw ADC, positive delta only (original HIDtaiko method). A piezo
// swings positive then negative when hit; counting only the rising
// delta means the negative ringing swing can never become a hit.
static const int32_t SMART_GATE = 18; // noise gate, subtracted as in v0.9

// Ratio-gate collection window state
static bool     win_open = false;
static uint32_t win_start_ms = 0;
static uint8_t  win_candidates = 0;   // bitmask of channels over threshold
static uint16_t win_peak[4] = {0};

uint8_t read_piezo_hits() {
  static uint64_t last_sample_us = 0;
  uint64_t now_us = to_us_since_boot(get_absolute_time());
  if (now_us - last_sample_us < 10) return 0;
  last_sample_us = now_us;

  build_profile_from_kando(piezo_profile);
  uint32_t now_ms = (uint32_t)millis();
  uint8_t immediate_hits = 0;

  for (int physical_i = 0; physical_i < 4; physical_i++) {
    // Physical pin order is reversed against the logical order
    int logical_i = 3 - physical_i;

    adc_select_input(3 - physical_i);
    long int raw_adc = (long int)adc_read();

    int32_t raw_delta = (int32_t)(raw_adc - prev_raw[logical_i]);
    prev_raw[logical_i] = raw_adc;

    // Positive only: a falling / ringing-negative swing is not a hit
    int32_t delta = 0;
    if (raw_delta > SMART_GATE) delta = raw_delta - SMART_GATE;

    last_deltas[logical_i] = delta;
    if (delta > (long int)delta_max[logical_i]) delta_max[logical_i] = (uint16_t)delta;
    if (delta > (int32_t)raw_peak[logical_i]) raw_peak[logical_i] = (uint16_t)(delta > 65535 ? 65535 : delta);

    if (settingMode) continue; // baseline (prev_raw) still updates above
    if (delta <= (int32_t)piezo_profile.threshold[logical_i]) continue;

    if (kando[13] <= 0) {
      // Immediate mode (JUDGE_WIN=0): v0.9-style decision, positive delta
      if (piezo_gate.accept((uint8_t)logical_i, delta, now_ms, piezo_profile)) {
        immediate_hits |= (1 << logical_i);
      }
    } else {
      // Window mode: collect candidates and their peaks first
      if (!win_open) {
        win_open = true;
        win_start_ms = now_ms;
        win_candidates = 0;
        for (int i = 0; i < 4; i++) win_peak[i] = 0;
      }
      win_candidates |= (1 << logical_i);
      if (delta > (int32_t)win_peak[logical_i]) {
        win_peak[logical_i] = (uint16_t)(delta > 65535 ? 65535 : delta);
      }
    }
  }

  if (settingMode) {
    win_open = false;
    win_candidates = 0;
    return 0;
  }
  if (kando[13] <= 0) return immediate_hits;
  if (!win_open) return 0;
  if (now_ms - win_start_ms < (uint32_t)kando[13]) return 0; // still collecting

  // Window closed: decide. Strongest peak first, so it claims the
  // pair-delay timers before weaker (crosstalk) candidates are judged.
  uint8_t order[4] = {0, 1, 2, 3};
  for (int a = 0; a < 3; a++) {
    for (int b = a + 1; b < 4; b++) {
      if (win_peak[order[b]] > win_peak[order[a]]) {
        uint8_t tmp = order[a]; order[a] = order[b]; order[b] = tmp;
      }
    }
  }
  uint16_t strongest = win_peak[order[0]];
  uint8_t hits = 0;
  for (int n = 0; n < 4; n++) {
    uint8_t ch = order[n];
    if ((win_candidates & (1 << ch)) == 0) continue;
    if (n > 0) {
      // Ratio gate: a candidate much weaker than the strongest hit in
      // the same window is body crosstalk, whatever its absolute size.
      // It must reach CROSS_RATIO % of the strongest peak to survive.
      uint32_t scaled = (uint32_t)win_peak[ch] * 100u;
      if (scaled < (uint32_t)strongest * (uint32_t)kando[12]) continue;
    }
    if (piezo_gate.accept(ch, (int32_t)win_peak[ch], now_ms, piezo_profile)) {
      hits |= (1 << ch);
    }
  }
  win_open = false;
  win_candidates = 0;
  return hits;
}

// ---- Settings (EEPROM, profile-separated) ----------------------------
void saveSettings() {
  for (int i = 0; i < KANDO_COUNT; i++) profiles[activeProfile][i] = kando[i];
  for (int p = 0; p < 2; p++) {
    int base = (p == PROFILE_PC) ? EE_OFF_PC : EE_OFF_SW;
    for (int i = 0; i < KANDO_COUNT; i++) {
      int v = profiles[p][i];
      if (v < 0) v = 0;
      if (v > kandoMax[i]) v = kandoMax[i];
      EEPROM.write(base + i, (uint8_t)v);
    }
  }
  EEPROM.write(EE_MAGIC_ADDR, EE_MAGIC);
  EEPROM.write(EE_MAGIC_ADDR + 1, EE_LAYOUT_VER);
  EEPROM.commit();
}

void loadSettings() {
  bool migrated = false;
  if (EEPROM.read(EE_MAGIC_ADDR) == EE_MAGIC &&
      EEPROM.read(EE_MAGIC_ADDR + 1) == EE_LAYOUT_VER) {
    for (int p = 0; p < 2; p++) {
      int base = (p == PROFILE_PC) ? EE_OFF_PC : EE_OFF_SW;
      for (int i = 0; i < KANDO_COUNT; i++) {
        uint8_t v = EEPROM.read(base + i);
        profiles[p][i] = (v == 0xFF) ? kandoDefault[i]
                                     : constrain((int)v, 0, kandoMax[i]);
      }
    }
  } else {
    // Migrate the v0.9 layout: single profile (12 values) at 0..11
    // becomes the starting point for both profiles.
    int legacy[KANDO_COUNT];
    bool hasLegacy = false;
    for (int i = 0; i < KANDO_COUNT; i++) legacy[i] = kandoDefault[i];
    for (int i = 0; i < 12; i++) {
      uint8_t v = EEPROM.read(i);
      if (v != 0xFF) { legacy[i] = constrain((int)v, 0, kandoMax[i]); hasLegacy = true; }
    }
    for (int p = 0; p < 2; p++) {
      for (int i = 0; i < KANDO_COUNT; i++) profiles[p][i] = legacy[i];
    }
    migrated = hasLegacy;
  }
  for (int i = 0; i < KANDO_COUNT; i++) kando[i] = profiles[activeProfile][i];
  if (migrated || EEPROM.read(EE_MAGIC_ADDR) != EE_MAGIC) saveSettings();
}

// ---------------- OLED display (128x64, 2-colour panel) ---------------
// Yellow band y=0..15 / gap y=16..19 (no light) / blue band y=20..63.
// Size-2 text is exactly 16 dots high, so headings sit in the yellow
// band and never straddle the colour boundary.
void drawCentered(const char *txt, int y, int size) {
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(size);
  display.getTextBounds(txt, 0, y, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - (int)w) / 2, y);
  display.print(txt);
}

#define OLED_BLUE_Y 20

void displayMode() {
  if (!oled_ok) return;
  bool sw = (currentUsbMode == MODE_SWITCH_TATACON);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawCentered(sw ? "SW MODE" : "PC MODE", 0, 2);                       // yellow
  drawCentered(sw ? "Switch Tatacon" : "Keyboard (D F J K)", 26, 1);    // blue
  drawCentered(sw ? "SLCT LONG: SETTING" : "SELECT: SETTING", 50, 1);   // blue
  display.display();
}

void displaySetting() {
  if (!oled_ok) return;
  char title[24];
  snprintf(title, sizeof(title), "SETTING %d/%d %s",
           currentSetting + 1, KANDO_COUNT,
           activeProfile == PROFILE_SW ? "SW" : "PC");
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawCentered(kandoNamesShort[currentSetting], 0, 2);  // yellow
  drawCentered(title, OLED_BLUE_Y + 1, 1);              // blue
  char val[8];
  snprintf(val, sizeof(val), "%d", kando[currentSetting]);
  drawCentered(val, 31, 3);                             // blue
  display.drawRect(0, 57, SCREEN_WIDTH, 7, SSD1306_WHITE);
  int fill = (kando[currentSetting] * (SCREEN_WIDTH - 4)) / kandoMax[currentSetting];
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
  activeProfile = (currentUsbMode == MODE_SWITCH_TATACON) ? PROFILE_SW : PROFILE_PC;

  if (currentUsbMode == MODE_SWITCH_TATACON) {
    Serial.end(); // Switch mode: no CDC serial, or the Switch will not recognise it
    serial_on = false;
    USBDevice.setID(SWITCH_VID, SWITCH_PID);
    USBDevice.setManufacturerDescriptor("HORI CO., LTD.");
    USBDevice.setProductDescriptor("HORIPAD");
    usb_hid.setReportDescriptor(desc_hid_report_joystick, sizeof(desc_hid_report_joystick));
    pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // blue: Switch mode
  } else {
    USBDevice.setID(0xCAFE, 0x4004);
    usb_hid.setReportDescriptor(desc_hid_report_keyboard, sizeof(desc_hid_report_keyboard));
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // green: keyboard mode
  }
  pixels.show();

  usb_hid.setPollInterval(2);
  usb_hid.begin();

  if (currentUsbMode == MODE_SWITCH_TATACON) {
    uint32_t wait_start = millis();
    while (!USBDevice.mounted() && millis() - wait_start < 3000) { delay(10); }
    delay(500);
  } else {
    Serial.begin(115200); // PC mode only, after USB init
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
    tmp[sizeof(tmp) - 1] = '\0';
    char *tok = strtok(tmp, ":");
    while (tok) {
      char *eq = strchr(tok, '=');
      if (eq) {
        *eq = '\0';
        int k = atoi(tok);
        int v = atoi(eq + 1);
        if (k >= 0 && k < KANDO_COUNT) kando[k] = constrain(v, 0, kandoMax[k]);
      }
      tok = strtok(NULL, ":");
    }
    Serial.println("OK:SENS");
  } else if (strcmp(line, "GETS") == 0) {
    Serial.print("--- CURRENT KANDO PARAMETERS (PROFILE: ");
    Serial.print(activeProfile == PROFILE_SW ? "SW" : "PC");
    Serial.println(") ---");
    for (int i = 0; i < KANDO_COUNT; i++) {
      Serial.print(i); Serial.print(" ["); Serial.print(kandoNames[i]); Serial.print("]: ");
      Serial.println(kando[i]);
    }
    Serial.println("OK:GETS");
  } else if (strcmp(line, "SAVE") == 0) {
    saveSettings();
    Serial.println("OK:SAVE");
  } else if (strcmp(line, "COPY2SW") == 0) {
    // Copy the profile being tuned on PC into the Switch profile
    for (int i = 0; i < KANDO_COUNT; i++) profiles[PROFILE_SW][i] = kando[i];
    saveSettings();
    Serial.println("OK:COPY2SW");
  } else if (strcmp(line, "COPY2PC") == 0) {
    for (int i = 0; i < KANDO_COUNT; i++) kando[i] = profiles[PROFILE_SW][i];
    saveSettings();
    Serial.println("OK:COPY2PC");
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

  // RAW waveform: every 40 ms, the peak delta of that interval
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
  displayMode();
}

void enterSetting() {
  settingMode = true;
  currentSetting = 0;
  displaySetting();
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // red: setting mode
  pixels.show();
  if (serial_on) Serial.println("--- MANUAL SETTING MODE ENTERED ---");
  logSetting("[SETTING] Current -> Index ");
}

void exitSetting() {
  settingMode = false;
  saveSettings();
  setLedNormal();
  if (serial_on) Serial.println("[SETTING] Saved & Exited to Normal Mode.");
}

void loop() {
  if (serial_on) serial_task();

  if (!usb_hid.ready()) { delay(1); return; }

  uint16_t s = readSNES();
  uint32_t now_loop = millis();

  // Piezo baseline keeps updating in setting mode (judgement is paused)
  uint8_t piezo_hits = read_piezo_hits();

  // Setting-mode entry/exit:
  //  PC mode:     Select tap toggles (same as v0.9)
  //  Switch mode: Select long-press (1 s) toggles; a short press is "-"
  static bool prev_select = false;
  static uint32_t select_press_ms = 0;
  static bool select_long_done = false;
  bool now_select = (s & (1 << SNES_SELECT)) != 0;

  if (now_select && !prev_select) select_press_ms = now_loop;
  if (!now_select) select_long_done = false;

  bool toggle_request = false;
  if (currentUsbMode == MODE_KEYBOARD_P1) {
    if (now_select && !prev_select) toggle_request = true;
  } else {
    if (now_select && !select_long_done &&
        now_loop - select_press_ms >= 1000) {
      toggle_request = true;
      select_long_done = true;
    }
  }
  if (toggle_request) {
    if (!settingMode) enterSetting(); else exitSetting();
  }
  prev_select = now_select;

  if (settingMode) {
    static bool last_up = false, last_down = false, last_a = false, last_b = false;
    bool up_now   = (s & (1 << SNES_UP)) != 0;
    bool down_now = (s & (1 << SNES_DOWN)) != 0;
    bool a_now    = (s & (1 << SNES_A)) != 0;
    bool b_now    = (s & (1 << SNES_B)) != 0;

    if (up_now && !last_up) {
      if (kando[currentSetting] < kandoMax[currentSetting]) kando[currentSetting]++;
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
        exitSetting();
        if (serial_on) Serial.println("[SETTING] Completed & Saved to EEPROM.");
      } else {
        displaySetting();
        logSetting("[SETTING] Next -> Index ");
      }
    }
    if (b_now && !last_b) {
      exitSetting();
      if (serial_on) Serial.println("[SETTING] Saved & Exited via B Button.");
    }
    last_up = up_now; last_down = down_now; last_a = a_now; last_b = b_now;
    return;
  }

  // Hit events for the Web monitor (PC mode only)
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

  // Keyboard mode (D, F, J, K): non-blocking, same HOLD_MS feel as Switch
  if (currentUsbMode == MODE_KEYBOARD_P1) {
    static uint8_t last_sent_hold = 0;
    if (piezo_hold != last_sent_hold) {
      uint8_t key_map[4] = {HID_KEY_D, HID_KEY_F, HID_KEY_J, HID_KEY_K};
      uint8_t rep[8] = {0};
      int slot = 2;
      for (int i = 0; i < 4 && slot < 8; i++) {
        if (piezo_hold & (1 << i)) rep[slot++] = key_map[i];
      }
      usb_hid.sendReport(REPORT_ID_KEYBOARD, rep, sizeof(rep));
      last_sent_hold = piezo_hold;
    }
    delay(2);
    return;
  }

  // Switch Tatacon mode report (layout unchanged from v0.9)
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

  // Edge (Ka) -> ZL / ZR (index 0: left Ka = D, index 3: right Ka = K)
  if (piezo_hold & (1 << 0)) report.buttons |= (1 << 6); // ZL
  if (piezo_hold & (1 << 3)) report.buttons |= (1 << 7); // ZR

  if (s & (1 << SNES_SELECT)) report.buttons |= (1 << 8); // -
  if (s & (1 << SNES_START))  report.buttons |= (1 << 9); // +

  // Face (Don) -> L3 / R3 (index 1: left Don = F, index 2: right Don = J)
  if (piezo_hold & (1 << 1)) report.buttons |= (1 << 10); // L3
  if (piezo_hold & (1 << 2)) report.buttons |= (1 << 11); // R3

  if (s & (1 << SNES_SELECT) && s & (1 << SNES_START)) {
    report.buttons |= (1 << 12); // HOME
  }

  report.hat = getHidHat(s);
  report.x = 2048; report.y = 2048; report.z = 2048; report.rz = 2048;

  usb_hid.sendReport(REPORT_ID_JOYSTICK, &report, sizeof(report));
  delay(2);
}
