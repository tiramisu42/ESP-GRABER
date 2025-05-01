#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <GyverButton.h>
#include "interface.h"

// Pin definitions for ESP32
#define OLED_SCL 22
#define OLED_SDA 21
#define BTN_UP 27   // K1 (Up)
#define BTN_DOWN 26 // K2 (Down)
#define BTN_OK 33   // K3 (OK)
#define BTN_BACK 32 // K4 (Back)
#define CC1101_GDO0 2
#define CC1101_CS 5
#define CC1101_SCK 18
#define CC1101_MOSI 23
#define CC1101_MISO 19

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Button configuration
GButton btn_up(BTN_UP, HIGH_PULL, NORM_OPEN);
GButton btn_down(BTN_DOWN, HIGH_PULL, NORM_OPEN);
GButton btn_ok(BTN_OK, HIGH_PULL, NORM_OPEN);
GButton btn_back(BTN_BACK, HIGH_PULL, NORM_OPEN);

// CC1101 configuration
#define RF_FREQUENCY 433.92 // MHz

// Constants
#define maxDataLog 160
#define minPause 1250
#define maxPause 6500
#define MAX_KEY_COUNT 50 // Maximum number of keys

// Data structures
volatile bool recieved = false;
volatile int keyRawLog[maxDataLog];
volatile byte logLen;
volatile bool sleepOn = false;

enum emKeys { kUnknown, kP12bt, k12bt, k24bt, k64bt, kKeeLoq, kANmotors64, kPrinceton };
enum emMenuState { menuLogo, menuMain, menuReceive, menuTransmit } menuState = menuLogo;

struct tpKeyRawData {
  byte keyID[9];
  int zero[2];
  int one[2];
  int prePulse[2];
  int startPause[2];
  int midlePause[2];
  byte prePulseLenth;
  byte codeLenth;
  byte firstDataIdx;
  emKeys type;
  byte rawDataLenth;
  int rawData[maxDataLog];
};

struct tpKeyData {
  byte keyID[9];
  int zero[2];
  int one[2];
  int prePulse[2];
  int startPause[2];
  int midlePause[2];
  byte prePulseLenth;
  byte codeLenth;
  byte firstDataIdx;
  emKeys type;
} keyData1;

byte maxKeyCount = MAX_KEY_COUNT;
byte EEPROM_key_count;
byte EEPROM_key_index = 0;
unsigned long stTimer = 0;
byte menuIndex = 0;
bool awaitingDeleteConfirmation = false;
bool validKeyReceived = false;

// Function prototypes
void ICACHE_RAM_ATTR handleInt();
String getTypeName(emKeys tp);
void OLED_printKey(tpKeyData* kd, byte msgType = 0, bool isSending = false);
void OLED_printError(String st, bool err = true);
byte indxKeyInROM(tpKeyData* kd);
bool EPPROM_AddKey(tpKeyData* kd);
void EEPROM_get_key(byte EEPROM_key_index1, tpKeyData* kd);
bool convert2Key(tpKeyData* kd);
bool convert2KeyRaw(tpKeyRawData* kd);
void sendSynthKey(tpKeyData* kd);
void deleteCurrentKey();
void OLED_printWaitingSignal();
void myDelayMcs(unsigned long dl);
void sendSynthBit(int bt[2]);

void setup() {
  Serial.begin(115200);

  // Initialize buttons
  btn_up.setDebounce(50);
  btn_up.setTimeout(500);
  btn_up.setClickTimeout(300);
  btn_up.setStepTimeout(200);
  btn_down.setDebounce(50);
  btn_down.setTimeout(500);
  btn_down.setClickTimeout(300);
  btn_down.setStepTimeout(200);
  btn_ok.setDebounce(50);
  btn_ok.setTimeout(500);
  btn_ok.setClickTimeout(300);
  btn_ok.setStepTimeout(200);
  btn_back.setDebounce(50);
  btn_back.setTimeout(500);
  btn_back.setClickTimeout(300);
  btn_back.setStepTimeout(200);
  btn_up.setTickMode(AUTO);
  btn_down.setTickMode(AUTO);

  // Initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  OLED_printLogo(display);

  // Initialize CC1101
  ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
  ELECHOUSE_cc1101.setGDO0(CC1101_GDO0);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(0); // ASK/OOK
  ELECHOUSE_cc1101.setMHZ(RF_FREQUENCY);
  ELECHOUSE_cc1101.SetRx();

  // Initialize EEPROM
  EEPROM.begin(512);
  byte read_count = EEPROM.read(0);
  EEPROM_key_count = (read_count < MAX_KEY_COUNT) ? read_count : MAX_KEY_COUNT;
  EEPROM_key_index = EEPROM.read(1);
  if (EEPROM_key_count > 0 && EEPROM_key_index <= EEPROM_key_count) {
    EEPROM_get_key(EEPROM_key_index, &keyData1);
  } else {
    EEPROM_key_count = 0;
    EEPROM_key_index = 0;
    EEPROM.write(0, 0);
    EEPROM.write(1, 0);
    EEPROM.commit();
    memset(&keyData1, 0, sizeof(tpKeyData));
  }

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), handleInt, CHANGE);

  // Wait for button press to exit logo
  while (!(btn_up.isClick() || btn_down.isClick() || btn_ok.isClick() || btn_back.isClick())) {
    btn_up.tick();
    btn_down.tick();
    btn_ok.tick();
    btn_back.tick();
  }
  menuState = menuMain;
  OLED_printMenu(display, menuIndex);
}

void loop() {
  btn_up.tick();
  btn_down.tick();
  btn_ok.tick();
  btn_back.tick();

  // Handle serial commands
  if (Serial.available()) {
    char echo = Serial.read();
    if (echo == 'e') {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print(F("EEPROM cleared success!"));
      display.display();
      EEPROM.write(0, 0);
      EEPROM.write(1, 0);
      EEPROM.commit();
      EEPROM_key_count = 0;
      EEPROM_key_index = 0;
      memset(&keyData1, 0, sizeof(tpKeyData));
      stTimer = millis();
    } else if (echo == 't' && menuState == menuTransmit && EEPROM_key_count > 0) {
      sendSynthKey(&keyData1);
      stTimer = millis();
    }
  }

  // Menu state handling
  if (menuState == menuLogo) {
    if (btn_up.isClick() || btn_down.isClick() || btn_ok.isClick() || btn_back.isClick()) {
      menuState = menuMain;
      OLED_printMenu(display, menuIndex);
    }
  } else if (menuState == menuMain) {
    if (btn_up.isClick()) {
      menuIndex = (menuIndex == 0) ? 1 : 0;
      OLED_printMenu(display, menuIndex);
    }
    if (btn_down.isClick()) {
      menuIndex = (menuIndex == 0) ? 1 : 0;
      OLED_printMenu(display, menuIndex);
    }
    if (btn_ok.isClick()) {
      menuState = (menuIndex == 0) ? menuReceive : menuTransmit;
      if (menuState == menuReceive) {
        ELECHOUSE_cc1101.SetRx();
        validKeyReceived = false;
        memset(&keyData1, 0, sizeof(tpKeyData));
        OLED_printWaitingSignal();
      } else {
        if (EEPROM_key_count > 0) {
          EEPROM_get_key(EEPROM_key_index, &keyData1);
        } else {
          memset(&keyData1, 0, sizeof(tpKeyData));
        }
        OLED_printKey(&keyData1);
      }
      stTimer = millis();
    }
    if (btn_back.isClick()) {
      menuState = menuLogo;
      OLED_printLogo(display);
    }
  } else {
    // Handle EEPROM clear
    if (btn_up.isHold() && btn_down.isHold()) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print(F("EEPROM cleared success!"));
      display.display();
      EEPROM.write(0, 0);
      EEPROM.write(1, 0);
      EEPROM.commit();
      EEPROM_key_count = 0;
      EEPROM_key_index = 0;
      memset(&keyData1, 0, sizeof(tpKeyData));
      stTimer = millis();
    }

    // Handle deletion confirmation
    if (awaitingDeleteConfirmation) {
      if (btn_ok.isClick()) {
        deleteCurrentKey();
        awaitingDeleteConfirmation = false;
        if (menuState == menuTransmit) {
          OLED_printKey(&keyData1);
        } else if (menuState == menuReceive) {
          OLED_printWaitingSignal();
        }
        stTimer = millis();
      } else if (btn_back.isClick() || btn_up.isClick() || btn_down.isClick()) {
        awaitingDeleteConfirmation = false;
        if (menuState == menuTransmit) {
          OLED_printKey(&keyData1);
        } else if (menuState == menuReceive) {
          OLED_printWaitingSignal();
        }
        stTimer = millis();
      }
      return;
    }

    // Menu-specific actions
    if (menuState == menuReceive) {
      if (btn_up.isClick() && (EEPROM_key_count > 0)) {
        EEPROM_key_index = (EEPROM_key_index == 1) ? EEPROM_key_count : EEPROM_key_index - 1;
        EEPROM_get_key(EEPROM_key_index, &keyData1);
        OLED_printKey(&keyData1);
        stTimer = millis();
      }
      if (btn_down.isClick() && (EEPROM_key_count > 0)) {
        EEPROM_key_index = (EEPROM_key_index == EEPROM_key_count) ? 1 : EEPROM_key_index + 1;
        EEPROM_get_key(EEPROM_key_index, &keyData1);
        OLED_printKey(&keyData1);
        stTimer = millis();
      }
      if (btn_ok.isHolded() && validKeyReceived) {
        if (EPPROM_AddKey(&keyData1)) {
          display.clearDisplay();
          display.drawBitmap(16, 6, image_DolphinSaved_bits, 92, 58, 1);
          display.setTextColor(1);
          display.setTextWrap(false);
          display.setCursor(6, 16);
          display.print("Saved");
          display.display();
          delay(1000);
          validKeyReceived = false;
        } else {
          OLED_printError(F("Key not saved"), true);
        }
        OLED_printWaitingSignal();
        stTimer = millis();
      }
      if (recieved) {
        if (convert2Key(&keyData1)) {
          validKeyReceived = true;
          display.clearDisplay();
          display.drawBitmap(-1, 33, image_Auth_bits, 62, 31, 1);
          display.display();
          delay(1000);
          byte idx = indxKeyInROM(&keyData1);
          OLED_printKey(&keyData1, idx == 0 ? 1 : 3);
        } else {
          OLED_printError(F("Key conversion failed"), true);
          validKeyReceived = false;
        }
        stTimer = millis();
        recieved = false;
      }
    } else if (menuState == menuTransmit) {
      if (btn_up.isClick() && (EEPROM_key_count > 0)) {
        EEPROM_key_index = (EEPROM_key_index == 1) ? EEPROM_key_count : EEPROM_key_index - 1;
        EEPROM_get_key(EEPROM_key_index, &keyData1);
        OLED_printKey(&keyData1);
        stTimer = millis();
      }
      if (btn_down.isClick() && (EEPROM_key_count > 0)) {
        EEPROM_key_index = (EEPROM_key_index == EEPROM_key_count) ? 1 : EEPROM_key_index + 1;
        EEPROM_get_key(EEPROM_key_index, &keyData1);
        OLED_printKey(&keyData1);
        stTimer = millis();
      }
      if (btn_ok.isClick()) {
        if (EEPROM_key_count > 0) {
          sendSynthKey(&keyData1);
        } else {
          OLED_printError(F("No keys to send"), true);
          delay(1000);
          OLED_printKey(&keyData1);
        }
        stTimer = millis();
      }
    }

    if (btn_back.isClick()) {
      menuState = menuMain;
      OLED_printMenu(display, menuIndex);
      stTimer = millis();
    }
    if (btn_back.isHold() && EEPROM_key_count > 0 && !awaitingDeleteConfirmation) {
      display.clearDisplay();
      display.drawBitmap(83, 22, image_WarningDolphinFlip_bits, 45, 42, 1);
      display.drawBitmap(77, 2, image_file_delete_bin_bits, 13, 16, 1);
      display.setTextColor(1);
      display.setTextWrap(false);
      display.setCursor(7, 7);
      display.print("Press OK to ");
      display.display();
      awaitingDeleteConfirmation = true;
      stTimer = millis();
    }
  }
}

void OLED_printWaitingSignal() {
  display.clearDisplay();
  display.drawBitmap(0, 4, image_RFIDDolphinReceive_bits, 97, 61, 1);
  display.drawBitmap(79, 8, image_Layer_4_bits, 23, 17, 0);
  display.drawBitmap(72, 11, image_External_ant_1_bits, 19, 11, 1);
  display.setTextColor(1);
  display.setTextWrap(false);
  display.setCursor(65, 38);
  display.print("Waiting");
  display.setCursor(65, 47);
  display.print("signal...");
  display.display();
}

void deleteCurrentKey() {
  if (EEPROM_key_count == 0 || EEPROM_key_index == 0) return;

  for (byte i = EEPROM_key_index; i < EEPROM_key_count; i++) {
    tpKeyData tempKey;
    EEPROM_get_key(i + 1, &tempKey);
    int address = i * sizeof(tpKeyData) + 2;
    for (byte j = 0; j < sizeof(tpKeyData); j++) {
      EEPROM.write(address + j, ((byte*)&tempKey)[j]);
    }
  }

  EEPROM_key_count--;
  EEPROM.write(0, EEPROM_key_count);
  if (EEPROM_key_index > EEPROM_key_count) {
    EEPROM_key_index = EEPROM_key_count;
  }
  EEPROM.write(1, EEPROM_key_index);
  EEPROM.commit();

  if (EEPROM_key_count > 0) {
    EEPROM_get_key(EEPROM_key_index, &keyData1);
  } else {
    memset(&keyData1, 0, sizeof(tpKeyData));
  }
  display.clearDisplay();
  display.drawBitmap(5, 2, image_DolphinMafia_bits, 119, 62, 1);
  display.setTextColor(1);
  display.setTextWrap(false);
  display.setCursor(84, 15);
  display.print("Deleted");
  display.display();
  delay(1000);
}

void OLED_printKey(tpKeyData* kd, byte msgType, bool isSending) {
  String st;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setTextWrap(false);
  
  switch (msgType) {
    case 0:
      st = " Signal " + String(EEPROM_key_index) + "/" + String(EEPROM_key_count) + " in ESP";
      break;
    case 1:
      st = "Hold OK to save";
      break;
    case 3:
      st = " Signal " + String(indxKeyInROM(kd)) + " exists in ESP";
      break;
  }
  display.setCursor(0, 0);
  display.print(st);
  st = "";
  for (byte i = 0; i < kd->codeLenth >> 3; i++) st += String(kd->keyID[i], HEX) + ":";
  display.setCursor(0, 12);
  display.print(st);
  st = "Type: " + getTypeName(kd->type);
  display.setCursor(0, 24);
  display.print(st);
  
  if (isSending) {
    display.setCursor(15, 53);
    display.print("Sending...");
    display.drawBitmap(55, 40, image_satellite_dish_bits, 15, 16, 1);
  }
  
  display.display();
}

void OLED_printError(String st, bool err) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(err ? F("Error!") : F("OK"));
  display.setCursor(0, 12);
  display.print(st);
  display.display();
}

byte indxKeyInROM(tpKeyData* kd) {
  bool eq = true;
  byte* buf = (byte*)kd;
  for (byte j = 1; j <= EEPROM_key_count; j++) {
    byte i = (kd->type == kKeeLoq || kd->type == kANmotors64) ? 4 : 0;
    for (; i < kd->codeLenth >> 3; i++) {
      if (buf[i] != EEPROM.read(i + j * sizeof(tpKeyData) + 2)) {
        eq = false;
        break;
      }
    }
    if (eq) return j;
    eq = true;
  }
  return 0;
}

bool EPPROM_AddKey(tpKeyData* kd) {
  byte indx = indxKeyInROM(kd);
  if (indx != 0) {
    EEPROM_key_index = indx;
    EEPROM.write(1, EEPROM_key_index);
    EEPROM.commit();
    return false;
  }
  if (EEPROM_key_count >= maxKeyCount) return false;
  EEPROM_key_count++;
  EEPROM_key_index = EEPROM_key_count;
  int address = EEPROM_key_index * sizeof(tpKeyData) + 2;
  for (byte i = 0; i < sizeof(tpKeyData); i++) {
    EEPROM.write(address + i, ((byte*)kd)[i]);
  }
  EEPROM.write(0, EEPROM_key_count);
  EEPROM.write(1, EEPROM_key_index);
  EEPROM.commit();
  return true;
}

void EEPROM_get_key(byte EEPROM_key_index1, tpKeyData* kd) {
  int address = EEPROM_key_index1 * sizeof(tpKeyData) + 2;
  if (address > EEPROM.length()) return;
  for (byte i = 0; i < sizeof(tpKeyData); i++) {
    ((byte*)kd)[i] = EEPROM.read(address + i);
  }
}

void ICACHE_RAM_ATTR handleInt() {
  static byte changeCnt = 0;
  static byte repeatCnt = 0;
  if (recieved) {
    repeatCnt = 0;
    changeCnt = 0;
    return;
  }
  static unsigned long lastTime = 0;
  const unsigned long curTime = micros();
  const int duration = curTime - lastTime;
  lastTime = curTime;

  if (((duration < 300) || (duration > 12000)) && (changeCnt == 0)) return;
  if ((duration > 300) && (duration < 12000) && (changeCnt >= 24)) {
    const int delta = duration - abs(keyRawLog[0]);
    if (abs(delta) < 100) {
      repeatCnt++;
      if (repeatCnt >= 2) {
        recieved = true;
        logLen = changeCnt;
        repeatCnt = 0;
        changeCnt = 0;
        return;
      }
      changeCnt = 0;
    }
  }
  if ((duration > 300) && (duration < 12000) && (changeCnt > 0) && (changeCnt < 20)) {
    changeCnt = 0;
    repeatCnt = 0;
  }
  if ((repeatCnt > 0) && (abs(duration - abs(keyRawLog[changeCnt])) > 50)) {
    changeCnt = 0;
    repeatCnt = 0;
    return;
  }
  if (!digitalRead(CC1101_GDO0)) {
    keyRawLog[changeCnt] = duration;
  } else {
    keyRawLog[changeCnt] = -duration;
  }
  changeCnt++;
  if (changeCnt >= maxDataLog) {
    changeCnt = 0;
    repeatCnt = 0;
  }
}

String getTypeName(emKeys tp) {
  switch (tp) {
    case kUnknown: return F(" Unknown");
    case kP12bt: return F(" Pre 12bit");
    case k12bt: return F(" 12bit");
    case k24bt: return F(" 24bit");
    case k64bt: return F(" 64bit");
    case kKeeLoq: return F(" KeeLoq");
    case kANmotors64: return F(" ANmotors");
    case kPrinceton: return F(" Princeton");
  }
  return "";
}

bool convert2Key(tpKeyData* kd) {
  long zero[2] = {0, 0}, one[2] = {0, 0};
  kd->prePulseLenth = 0;
  kd->startPause[0] = keyRawLog[logLen - 1];
  kd->startPause[1] = keyRawLog[0];
  kd->midlePause[0] = 0;
  kd->midlePause[1] = 0;
  byte i = 1, k = 0, k0 = 0, k1 = 0, j = 0;

  bool isPrinceton = abs(kd->startPause[0]) > 8000 && abs(kd->startPause[0]) < 12000;

  if (isPrinceton) {
    kd->prePulseLenth = 0;
    kd->firstDataIdx = i;
    kd->codeLenth = (logLen - i) >> 1;

    for (; i < logLen; i += 2) {
      int high = abs(keyRawLog[i]);
      int low = abs(keyRawLog[i + 1]);
      if (high > 800 && high < 1200 && low > 200 && low < 600) {
        bitSet(kd->keyID[k >> 3], 7 - j);
        one[0] += keyRawLog[i];
        one[1] += keyRawLog[i + 1];
        k1++;
      } else if (high > 200 && high < 600 && low > 800 && low < 1200) {
        bitClear(kd->keyID[k >> 3], 7 - j);
        zero[0] += keyRawLog[i];
        zero[1] += keyRawLog[i + 1];
        k0++;
      } else {
        return false;
      }
      j++;
      if (j > 7) j = 0;
      k++;
      if (k >= kd->codeLenth) break;
    }
  } else {
    unsigned int halfT = (abs(keyRawLog[i]) + abs(keyRawLog[i + 1])) >> 1;
    if (logLen > 131) {
      for (; i < logLen; i++) {
        if (abs(keyRawLog[i]) > 500) break;
        one[0] += keyRawLog[i];
        i++;
        if (abs(keyRawLog[i]) > 500) break;
        one[1] += keyRawLog[i];
      }
      if (i > 100) return false;
      kd->prePulseLenth = i - 2;
      kd->midlePause[0] = keyRawLog[i - 1];
      kd->midlePause[1] = keyRawLog[i];
      i++;
      kd->prePulse[0] = (one[0] << 1) / kd->prePulseLenth;
      kd->prePulse[1] = (one[1] << 1) / kd->prePulseLenth;
      one[0] = 0;
      one[1] = 0;
    }
    kd->firstDataIdx = i;
    kd->codeLenth = (logLen - i) >> 1;
    halfT = (abs(keyRawLog[i]) + abs(keyRawLog[i + 1])) >> 1;
    for (; i < logLen; i += 2) {
      if (abs(keyRawLog[i]) > halfT) {
        bitSet(kd->keyID[k >> 3], 7 - j);
        one[0] += keyRawLog[i];
        one[1] += keyRawLog[i + 1];
        k1++;
      } else {
        bitClear(kd->keyID[k >> 3], 7 - j);
        zero[0] += keyRawLog[i];
        zero[1] += keyRawLog[i + 1];
        k0++;
      }
      j++;
      if (j > 7) j = 0;
      k++;
      if (k >= kd->codeLenth) break;
    }
  }

  if (k1 == 0 || k0 == 0) return false;
  kd->one[0] = one[0] / k1;
  kd->one[1] = one[1] / k1;
  kd->zero[0] = zero[0] / k0;
  kd->zero[1] = zero[1] / k0;

  if (isPrinceton) {
    kd->type = kPrinceton;
  } else {
    switch (kd->codeLenth) {
      case 12:
        kd->type = (kd->prePulseLenth == 0) ? k12bt : kP12bt;
        break;
      case 24:
        kd->type = k24bt;
        break;
      case 64:
        kd->type = k64bt;
        break;
      case 65:
        kd->type = (kd->keyID[2] == kd->keyID[3]) ? kANmotors64 : kKeeLoq;
        break;
      default:
        kd->type = kUnknown;
    }
  }
  return true;
}

bool convert2KeyRaw(tpKeyRawData* kd) {
  kd->rawDataLenth = logLen;
  for (byte i = 0; i < logLen; i++) kd->rawData[i] = keyRawLog[i];
  return convert2Key((tpKeyData*)kd);
}

void myDelayMcs(unsigned long dl) {
  if (dl > 16000) {
    delay(dl / 1000);
  } else {
    delayMicroseconds(dl);
  }
}

void sendSynthKey(tpKeyData* kd) {
  recieved = true;
  OLED_printKey(kd, 0, true);

  ELECHOUSE_cc1101.setModulation(0);
  ELECHOUSE_cc1101.setMHZ(RF_FREQUENCY);
  ELECHOUSE_cc1101.SetTx();
  randomSeed(millis());
  byte ANmotorsByte = random(256);
  for (byte k = 0; k < 4; k++) {
    sendSynthBit(kd->startPause);
    if (kd->prePulseLenth > 0) {
      for (byte i = 0; i < (kd->prePulseLenth) >> 1; i++)
        sendSynthBit(kd->prePulse);
    }
    sendSynthBit(kd->midlePause);
    byte j = 0, bt;
    for (byte i = 0; i < kd->codeLenth; i++) {
      if (((i >> 3) >= 2) && ((i >> 3) <= 3) && (kd->type == kANmotors64))
        bt = 1 & (ANmotorsByte >> (7 - j));
      else
        bt = 1 & (kd->keyID[i >> 3] >> (7 - j));
      if (bt)
        sendSynthBit(kd->one);
      else
        sendSynthBit(kd->zero);
      j++;
      if (j > 7) j = 0;
    }
  }
  ELECHOUSE_cc1101.SetRx();
  recieved = false;

  display.clearDisplay();
  display.setTextColor(1);
  display.setTextWrap(false);
  display.setCursor(15, 47);
  display.print("Successfully!");
  display.drawBitmap(15, 12, image_Connected_bits, 62, 31, 1);
  display.display();
  delay(1000);
  OLED_printKey(kd);
}

void sendSynthBit(int bt[2]) {
  static uint8_t dataOn[1] = {0xFF};
  static uint8_t dataOff[1] = {0x00};
  if (bt[0] == 0) return;
  for (byte i = 0; i < 2; i++) {
    if (bt[i] > 0) {
      ELECHOUSE_cc1101.SendData(dataOn, 1);
      myDelayMcs(bt[i]);
    } else {
      ELECHOUSE_cc1101.SendData(dataOff, 1);
      myDelayMcs(-bt[i]);
    }
  }
}
