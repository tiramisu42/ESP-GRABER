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

byte maxKeyCount = EEPROM.length() / sizeof(tpKeyData);
byte EEPROM_key_count;
byte EEPROM_key_index = 0;
unsigned long stTimer = 0;
byte menuIndex = 0;
bool awaitingDeleteConfirmation = false;

// Function prototypes
String getTypeName(emKeys tp);
void OLED_printKey(tpKeyData* kd, byte msgType = 0, bool isSending = false);
void OLED_printError(String st, bool err = true);
byte indxKeyInROM(tpKeyData* kd);
bool EPPROM_AddKey(tpKeyData* kd);
void EEPROM_get_key(byte EEPROM_key_index1, tpKeyData* kd);
bool convert2Key(tpKeyData* kd);
bool convert2KeyRaw(tpKeyRawData* kd);
void sendSynthKey(tpKeyData* kd);
void printDebugData();
void deleteCurrentKey();

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect

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
  ELECHOUSE_cc1101.setModulation(0); // ASK/OOK for Princeton
  ELECHOUSE_cc1101.setMHZ(RF_FREQUENCY);
  ELECHOUSE_cc1101.SetRx();

  // Initialize EEPROM
  EEPROM.begin(512);
  EEPROM_key_count = EEPROM.read(0);
  if (EEPROM_key_count > maxKeyCount) EEPROM_key_count = 0;
  if (EEPROM_key_count != 0) {
    EEPROM_key_index = EEPROM.read(1);
    Serial.print(F("Read key code from EEPROM: "));
    EEPROM_get_key(EEPROM_key_index, &keyData1);
    for (byte i = 0; i < 8; i++) {
      Serial.print(keyData1.keyID[i], HEX);
      Serial.print(F(":"));
    }
    Serial.println();
  }

  // Attach interrupt for CC1101 GDO0
  attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), handleInt, CHANGE);

  // Wait for button press to exit logo screen
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
      Serial.println(F("EEPROM cleared"));
      EEPROM.write(0, 0);
      EEPROM.write(1, 0);
      EEPROM.commit();
      EEPROM_key_count = 0;
      EEPROM_key_index = 0;
      display.display();
      stTimer = millis();
    } else if (echo == 't' && menuState == menuTransmit) {
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
        OLED_printKey(&keyData1);
      } else {
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
      Serial.println(F("EEPROM cleared"));
      EEPROM.write(0, 0);
      EEPROM.write(1, 0);
      EEPROM.commit();
      EEPROM_key_count = 0;
      EEPROM_key_index = 0;
      display.display();
      stTimer = millis();
    }

    // Handle deletion confirmation
    if (awaitingDeleteConfirmation) {
      if (btn_ok.isClick()) {
        deleteCurrentKey();
        awaitingDeleteConfirmation = false;
        OLED_printKey(&keyData1);
        stTimer = millis();
      } else if (btn_back.isClick() || btn_up.isClick() || btn_down.isClick()) {
        awaitingDeleteConfirmation = false;
        OLED_printKey(&keyData1);
        stTimer = millis();
      }
      return; // Block other actions until confirmation is resolved
    }

    // Menu-specific actions
    if (menuState == menuReceive) {
      // Navigate keys
      if (btn_up.isClick() && (EEPROM_key_count > 0)) {
        EEPROM_key_index--;
        if (EEPROM_key_index < 1) EEPROM_key_index = EEPROM_key_count;
        EEPROM_get_key(EEPROM_key_index, &keyData1);
        OLED_printKey(&keyData1);
        stTimer = millis();
      }
      if (btn_down.isClick() && (EEPROM_key_count > 0)) {
        EEPROM_key_index++;
        if (EEPROM_key_index > EEPROM_key_count) EEPROM_key_index = 1;
        EEPROM_get_key(EEPROM_key_index, &keyData1);
        OLED_printKey(&keyData1);
        stTimer = millis();
      }

      // Save key
      if (btn_ok.isHolded()) {
        if (EPPROM_AddKey(&keyData1)) {
          display.clearDisplay();
          display.drawBitmap(16, 6, image_DolphinSaved_bits, 92, 58, 1);
          display.setTextColor(1);
          display.setTextWrap(false);
          display.setCursor(6, 16);
          display.print("Saved");
          display.display();
          Serial.println(F("Key saved to EEPROM"));
          delay(1000);
        } else {
          OLED_printError(F("Key not saved"), true);
          Serial.println(F("Failed to save key to EEPROM"));
        }
        OLED_printKey(&keyData1);
        stTimer = millis();
      }

      // Process received signal
      if (recieved) {
        Serial.println(F("Signal received, processing..."));
        if (convert2Key(&keyData1)) {
          Serial.println(F("Key converted successfully"));
          if (indxKeyInROM(&keyData1) == 0) {
            OLED_printKey(&keyData1, 1);
          } else {
            OLED_printKey(&keyData1, 3);
          }
          for (byte i = 0; i < keyData1.codeLenth >> 3; i++) {
            Serial.print(keyData1.keyID[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
        } else {
          OLED_printError(F("Key conversion failed"), true);
          Serial.println(F("Key conversion failed"));
        }
        stTimer = millis();
        recieved = false;
      }
    } else if (menuState == menuTransmit) {
      // Navigate keys
      if (btn_up.isClick() && (EEPROM_key_count > 0)) {
        EEPROM_key_index--;
        if (EEPROM_key_index < 1) EEPROM_key_index = EEPROM_key_count;
        EEPROM_get_key(EEPROM_key_index, &keyData1);
        OLED_printKey(&keyData1);
        stTimer = millis();
      }
      if (btn_down.isClick() && (EEPROM_key_count > 0)) {
        EEPROM_key_index++;
        if (EEPROM_key_index > EEPROM_key_count) EEPROM_key_index = 1;
        EEPROM_get_key(EEPROM_key_index, &keyData1);
        OLED_printKey(&keyData1);
        stTimer = millis();
      }

      // Send key
      if (btn_ok.isClick()) {
        sendSynthKey(&keyData1);
        stTimer = millis();
      }
    }

    // Exit to main menu
    if (btn_back.isClick()) {
      menuState = menuMain;
      OLED_printMenu(display, menuIndex);
      stTimer = millis();
    }

    // Initiate key deletion
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

// Function to delete the current key
void deleteCurrentKey() {
  if (EEPROM_key_count == 0 || EEPROM_key_index == 0) return;

  // Shift keys to overwrite the deleted key
  for (byte i = EEPROM_key_index; i < EEPROM_key_count; i++) {
    tpKeyData tempKey;
    EEPROM_get_key(i + 1, &tempKey);
    int address = i * sizeof(tpKeyData);
    for (byte j = 0; j < sizeof(tpKeyData); j++) {
      EEPROM.write(address + j, ((byte*)&tempKey)[j]);
    }
  }

  // Decrease key count and update EEPROM
  EEPROM_key_count--;
  EEPROM.write(0, EEPROM_key_count);
  if (EEPROM_key_index > EEPROM_key_count) {
    EEPROM_key_index = EEPROM_key_count;
  }
  EEPROM.write(1, EEPROM_key_index);
  EEPROM.commit();

  // Update display
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
  Serial.println(F("Key deleted from EEPROM"));
  delay(1000);
}

// OLED display functions
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
      st = "Hold the Btn to save";
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
  if (err) {
    display.print(F("Error!"));
  } else {
    display.print(F("OK"));
  }
  display.setCursor(0, 12);
  display.print(st);
  display.display();
}

byte indxKeyInROM(tpKeyData* kd) {
  bool eq = true;
  byte* buf = (byte*)kd;
  for (byte j = 1; j <= EEPROM_key_count; j++) {
    byte i = 0;
    if ((kd->type == kKeeLoq) || (kd->type == kANmotors64)) i = 4;
    for (; i < kd->codeLenth >> 3; i++) {
      if (buf[i] != EEPROM.read(i + j * sizeof(tpKeyData))) {
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
  if (EEPROM_key_count <= maxKeyCount) EEPROM_key_count++;
  if (EEPROM_key_count < maxKeyCount) EEPROM_key_index = EEPROM_key_count;
  else EEPROM_key_index++;
  if (EEPROM_key_index > EEPROM_key_count) EEPROM_key_index = 1;
  Serial.println(F("Adding to EEPROM"));
  for (byte i = 0; i < kd->codeLenth >> 3; i++) {
    Serial.print(kd->keyID[i], HEX);
    Serial.print(F(":"));
  }
  Serial.println();
  int address = EEPROM_key_index * sizeof(tpKeyData);
  for (byte i = 0; i < sizeof(tpKeyData); i++) {
    EEPROM.write(address + i, ((byte*)kd)[i]);
  }
  EEPROM.write(0, EEPROM_key_count);
  EEPROM.write(1, EEPROM_key_index);
  EEPROM.commit();
  return true;
}

void EEPROM_get_key(byte EEPROM_key_index1, tpKeyData* kd) {
  int address = EEPROM_key_index1 * sizeof(tpKeyData);
  if (address > EEPROM.length()) return;
  for (byte i = 0; i < sizeof(tpKeyData); i++) {
    ((byte*)kd)[i] = EEPROM.read(address + i);
  }
}

void handleInt() {
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
        Serial.println(F("Interrupt: Signal captured"));
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

  bool isPrinceton = false;
  if (abs(kd->startPause[0]) > 8000 && abs(kd->startPause[0]) < 12000) {
    isPrinceton = true;
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
        Serial.println(F("convert2Key: Invalid Princeton bit timing"));
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
      if (i > 100) {
        Serial.println(F("convert2Key: Invalid preamble length"));
        return false;
      }
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

  if (k1 == 0 || k0 == 0) {
    Serial.println(F("convert2Key: No valid bits detected"));
    return false;
  }
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
        Serial.println(F("convert2Key: Unknown key length"));
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
  // Display signal info with "Sending..." message
  OLED_printKey(kd, 0, true);

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

  // Display "Successfully!" with connected bitmap
  display.clearDisplay();
  display.setTextColor(1);
  display.setTextWrap(false);
  display.setCursor(15, 47);
  display.print("Successfully!");
  display.drawBitmap(15, 12, image_Connected_bits, 62, 31, 1);
  display.display();
  delay(1000); // Show success message for 1 second

  // Restore key display
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

void printDebugData() {
  Serial.print(F(" codeLenth "));
  Serial.print(keyData1.codeLenth);
  Serial.print(F(", firstDataIdx "));
  Serial.print(keyData1.firstDataIdx);
  Serial.print(F(", Key type: "));
  Serial.print(getTypeName(keyData1.type));
  Serial.print(F(", zero ["));
  Serial.print(keyData1.zero[0]);
  Serial.print(", ");
  Serial.print(keyData1.zero[1]);
  Serial.print(F("], one ["));
  Serial.print(keyData1.one[0]);
  Serial.print(", ");
  Serial.print(keyData1.one[1]);
  Serial.print("]");
  Serial.print(F(", startPause ["));
  Serial.print(keyData1.startPause[0]);
  Serial.print(", ");
  Serial.print(keyData1.startPause[1]);
  Serial.print("]");
  if (keyData1.prePulseLenth > 0) {
    Serial.print(F(", prePulseLenth "));
    Serial.print(keyData1.prePulseLenth);
    Serial.print(F(", prePulse ["));
    Serial.print(keyData1.prePulse[0]);
    Serial.print(", ");
    Serial.print(keyData1.prePulse[1]);
    Serial.print("]");
  }
  if (abs(keyData1.midlePause[0]) > 0) {
    Serial.print(F(", Header ["));
    Serial.print(keyData1.midlePause[0]);
    Serial.print(", ");
    Serial.print(keyData1.midlePause[1]);
    Serial.print("]");
  }
  Serial.println();
  for (byte i = 0; i < logLen; i++) {
    Serial.print(keyRawLog[i]);
    Serial.print(", ");
    btn_up.tick();
    btn_down.tick();
    btn_ok.tick();
    btn_back.tick();
  }
  Serial.print(F(" rawLen "));
  Serial.println(logLen);
}
