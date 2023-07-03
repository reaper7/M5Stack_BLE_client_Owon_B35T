/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * <Reaper7> wrote this code. As long as you retain this 
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
*/

// M5Stack_BLE_client_Owon_B35T.ino
// 2018 Reaper7 (tested on M5Stack)

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEUtils.h>

#include <M5Stack.h>                                                            // M5Stack lib >= 0.1.9
#include "M5StackUpdater.h"                                                     // https://github.com/tobozo/M5Stack-SD-Updater

#include "meter_graphics.h"

#define MYDEBUG
#ifdef MYDEBUG
#define DEBUG_MSG(...) Serial.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif

//#define BATMETERI2C
//#define FALSEMETER
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
const char* OWONNAME = "BDM";                                                   // OWON device name 
static BLEUUID serviceUUID("0000fff0-0000-1000-8000-00805f9b34fb");             // OWON service UUID
static BLEUUID charnotificationUUID("0000fff4-0000-1000-8000-00805f9b34fb");    // OWON notification characteristic UUID
static BLEUUID charwriteUUID("0000fff3-0000-1000-8000-00805f9b34fb");           // OWON write characteristic UUID

static BLEAddress *pServerAddress;
static BLERemoteCharacteristic* pRemoteCharacteristicNotify;
static BLERemoteCharacteristic* pRemoteCharacteristicWrite;

volatile boolean deviceBleConnected = false;                                    // flag BLE conneted
volatile boolean newBleData = false;                                            // flag "we get new data from meter"
static unsigned long lastBleNotify = 0;                                         // timestamp "last received data from meter" in ms
static unsigned long startBleScanning = 0;                                      // timestamp when ble scan is beginning in ms

const uint16_t maxWaitForNotify = 10000;                                        // max wait time for next notify from meter in ms
const uint16_t scanTime = 10;                                                   // BLE scan time in s

const uint8_t meterReplySize = 14;                                              // number of bytes in meter reply
char valuechar[meterReplySize];                                                 // meter reply buffer

const uint8_t rawBufferSize = 14;                                               // number of bytes for reading meter
char rawvalbuf[rawBufferSize];                                                  // meter reply raw buffer

const uint8_t replySizeNorm = 14;                                               // notify size for b35t
const uint8_t replySizePlus = 6;                                                // notify size for b35tPLUS

static boolean meterIsPlus = false;                                             // flag for meter b35tPLUS
static boolean meterPlusLowBat = false;                                         // meter b35tPLUS Low Bat

static boolean firstNotify = true;                                              // flag first notify after start or reconnect

//write to OWON
volatile boolean deviceBleWriteAvailable = false;                               // write to meter available
const uint8_t buttonsMax = 6;                                                   // max buttons available on meter
const char *btnName[] = { "SELECT", "RANGE", "HLD/LIG", "REL/BT", "HZ/DUTY", "MAX/MIN" };  // buttons name
uint8_t btnNumber = 0x01;                                                       // initial/current button number(code)
uint8_t btnShortPress = 0x01;                                                   // code for short press (one short press)
uint8_t btnLongPress = 0x00;                                                    // code for long press
const uint16_t btnLongPressTime = 1800;                                         // long button press time in ms
const uint8_t owonBtnArraySize = 0x02;                                          // buffer size for write to meter
uint8_t owonBtnArray[owonBtnArraySize] = {btnNumber, btnShortPress};            // output buffer for write to meter

#ifdef BATMETERI2C
// bat meter MAX17043 1-Cell Fuel Gauge
static uint16_t batReadEvery = 10000;                                           // read MAX17043 every 10000 sec
static unsigned long batNextReadTime = 0;                                       // time for next read MAX17043 in ms
const uint8_t MAX17043ADDR=0x36;                                                // MAX17043 i2c addr
const uint8_t MAX17043CMDADDR=0xFE;                                             // MAX17043 command register addr
const uint8_t MAX17043SOCADDR=0x04;                                             // MAX17043 soc data addr
const uint8_t MAX17043VCELLADDR=0x02;                                           // MAX17043 voltage data addr
#endif
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/*
14 bytes reply
-----------------------------------------------------------------------
|  1 |         2         |  3 |  4 |  5 |  6 |    7    |  8 |    9    |
-----------------------------------------------------------------------
|0x2b|0x33 0x36 0x32 0x33|0x20|0x34|0x31|0x00|0x40 0x80|0x24|0x0d 0x0a|
-----------------------------------------------------------------------
|  0 |  1    2    3    4 |  5 |  6 |  7 |  8 |  9   10 | 11 | 12   13 |
-----------------------------------------------------------------------
*/

/*
1:  + or - (dec 43 or 45)
    BYTE 0
*/
  #define REGPLUSMINUS    0x00
  #define FLAGPLUS        B00101011
  #define FLAGMINUS       B00101101

/*
2:  Value 0000-9999 in dec
    BYTE 1-4
*/
  #define REGDIG1         0x01
  #define REGDIG2         0x02
  #define REGDIG3         0x03
  #define REGDIG4         0x04

/*
3:  Just space (dec 32)
    BYTE 5
*/

/*
4:  Decimal point position
    - dec 48 no point
    - dec 49 after the first number
    - dec 50 after the second number
    - dec 52 after the third number
    BYTE 6
*/
  #define REGPOINT        0x06
  #define FLAGPOINT0      B00110000
  #define FLAGPOINT1      B00110001
  #define FLAGPOINT2      B00110010
  #define FLAGPOINT3      B00110100

/*
5:  AC or DC and Auto mode
    - dec 49 DC Auto mode / 51 HOLD
    - dec 41 AC Auto mode / 43 HOLD
    - dec 17 DC Manual mode / 19 HOLD
    - dec 09 AC Manual mode / 11 HOLD
    BYTE 7
*/
  #define REGMODE         0x07
  #define FLAGMODENONE    B00000000   //none
  #define FLAGMODEMAN     B00000001   //manual
  #define FLAGMODEHOLD    B00000010   //hold
  #define FLAGMODEREL     B00000100   //relative  
  #define FLAGMODEAC      B00001000   //ac
  #define FLAGMODEDC      B00010000   //dc
  #define FLAGMODEAUTO    B00100000   //auto
  #define FLAGMODECHECK   B11111111   //mask for check others

/*
6:  MIN MAX
    - dec  0 MIN MAX off
    - dec 16 MIN
    - dec 32 MAX
    BYTE 8
*/
  #define REGMINMAX       0x08
  #define FLAGMINMAXNONE  B00000000
  #define FLAGMIN         B00010000
  #define FLAGMAX         B00100000

/*
7:  Units
    - dec   2   0    % Duty
    - dec   0   1 Fahrenheit
    - dec   0   2 Grad
    - dec   0   4   nF
    - dec   0   8   Hz
    - dec   0  16  hFE
    - dec   0  32  Ohm
    - dec  32  32 kOhm
    - dec  16  32 MOhm
    - dec 128  64   uA
    - dec  64  64   mA
    - dec   0  64    A
    - dec  64 128   mV
    - dec   0 128    V
    BYTES 9 & 10
*/
  #define REGSCALE        0x09
  #define FLAGSCALEDUTY   B00000010
  #define FLAGSCALEDIODE  B00000100
  #define FLAGSCALEBUZZ   B00001000
  #define FLAGSCALEMEGA   B00010000  
  #define FLAGSCALEKILO   B00100000
  #define FLAGSCALEMILI   B01000000
  #define FLAGSCALEMICRO  B10000000
  
  #define REGUNIT         0x0a
  #define FLAGUNITNONE    B00000000
  #define FLAGUNITFAHR    B00000001
  #define FLAGUNITGRAD    B00000010
  #define FLAGUNITNF      B00000100
  #define FLAGUNITHZ      B00001000
  #define FLAGUNITHFE     B00010000
  #define FLAGUNITOHM     B00100000
  #define FLAGUNITAMP     B01000000
  #define FLAGUNITVOLT    B10000000

/*
8:  ???
*/

/*
9:  CR + LF
*/
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void drawIcon(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, const uint8_t* data, uint16_t color) {
  M5.Lcd.setBitmapColor(color, BACKGROUND);
  //M5.Lcd.pushImage((int32_t)x0, (int32_t)y0, (uint32_t)w, (uint32_t)h, const_cast<uint8_t*>(data), false);
  M5.Lcd.pushImage((int32_t)x0, (int32_t)y0, (uint32_t)w, (uint32_t)h, const_cast<uint8_t*>(data), 0, false);
}
//------------------------------------------------------------------------------
/*
void batMeterInit() {                                                           // reset MAX17043
  Wire.beginTransmission(MAX17043ADDR);
  Wire.write(MAX17043CMDADDR);
  //Wire.write(0x40);                                                           // quick start with restart fuel-gauge calculations
  Wire.write(0x54);                                                             // normal reset
  Wire.write(0x00);
  Wire.endTransmission();
  delay(200);
}
*/
//------------------------------------------------------------------------------
void batCheckDraw() {
  //based on soc
  double soc = 0;
#ifdef BATMETERI2C
  DEBUG_MSG("I: BAT check...");
  Wire.beginTransmission(MAX17043ADDR);                                         // get SoC
  Wire.write(MAX17043SOCADDR);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX17043ADDR, (uint8_t)2);
  soc = Wire.read() + (double) Wire.read() / 256;
  DEBUG_MSG(" SoC: %6.2f%%\n", soc);
  M5.Lcd.fillRect(WACCUPOSX + 2, TOPROWPOSY + 5, ACCUSCALW, ACCUSCALH, soc>75?COLORICONACCU:COLORNOTACTIVE);
  M5.Lcd.fillRect(WACCUPOSX + 5, TOPROWPOSY + 5, ACCUSCALW, ACCUSCALH, soc>50?COLORICONACCU:COLORNOTACTIVE);
  M5.Lcd.fillRect(WACCUPOSX + 8, TOPROWPOSY + 5, ACCUSCALW, ACCUSCALH, soc>25?COLORICONACCU:COLORNOTACTIVE);
  M5.Lcd.fillRect(WACCUPOSX + 11, TOPROWPOSY + 5, ACCUSCALW, ACCUSCALH, soc>5?COLORICONACCU:COLORNOTACTIVE);
  drawIcon(WACCUPOSX, TOPROWPOSY, ICONW, ICONH, ACCU_BMP, soc>0?COLORICONACCU:COLORNOTACTIVE);
#else
  if (meterIsPlus) {                                                            // if meter is b35tPLUS
    if (meterPlusLowBat)                                                        // if BAT LOW
      soc = 1;
  }
  drawIcon(WACCUPOSX, TOPROWPOSY, ICONW, ICONH, ACCU_BMP, soc>0?COLORICONBAT:COLORICONACCU);
#endif
/*
  //based on voltage
  double volt = 0;
  DEBUG_MSG("I: BAT check...");
  Wire.beginTransmission(MAX17043ADDR);                                         // get voltage
  Wire.write(MAX17043VCELLADDR);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX17043ADDR, (uint8_t)2);
  volt = ( (Wire.read() << 4) + (Wire.read() >> 4) ) * 0.00125;
  DEBUG_MSG(" Volt: %4.2fV\n", volt);
  // >3.3 = 1
  // >3.5 = 2
  // >3.7 = 3
  // >3.9 = 4
  M5.Lcd.fillRect(WACCUPOSX + 2, TOPROWPOSY + 5, ACCUSCALW, ACCUSCALH, volt>3.9?TFT_GREEN:COLORNOTACTIVE);
  M5.Lcd.fillRect(WACCUPOSX + 5, TOPROWPOSY + 5, ACCUSCALW, ACCUSCALH, volt>3.7?TFT_GREEN:COLORNOTACTIVE);
  M5.Lcd.fillRect(WACCUPOSX + 8, TOPROWPOSY + 5, ACCUSCALW, ACCUSCALH, volt>3.5?TFT_GREEN:COLORNOTACTIVE);
  M5.Lcd.fillRect(WACCUPOSX + 11, TOPROWPOSY + 5, ACCUSCALW, ACCUSCALH, volt>3.3?TFT_GREEN:COLORNOTACTIVE);
  drawIcon(WACCUPOSX, TOPROWPOSY, ICONW, ICONH, ACCU_BMP, volt>0?COLORICONACCU:COLORNOTACTIVE);
*/
}
//------------------------------------------------------------------------------
void drawButtons() {
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(BACKGROUND);
  
  M5.Lcd.fillRoundRect(BTNAXPOS, M5.Lcd.height()-BTNYPOSFROMBOTTOM, BTNAW, BTNH, 3, deviceBleWriteAvailable == true?FONTCOLORVALUE:COLORNOTACTIVE);  
  M5.Lcd.fillRoundRect(BTNBXPOS, M5.Lcd.height()-BTNYPOSFROMBOTTOM, BTNBW, BTNH, 3, deviceBleWriteAvailable == true?FONTCOLORVALUE:COLORNOTACTIVE);  
  M5.Lcd.fillRoundRect(BTNCXPOS, M5.Lcd.height()-BTNYPOSFROMBOTTOM, BTNCW, BTNH, 3, deviceBleWriteAvailable == true?FONTCOLORVALUE:COLORNOTACTIVE);

  M5.Lcd.drawCentreString("<", BTNAXPOS + (BTNAW/2), M5.Lcd.height() - BTNYTEXTPOSFROMBOTTOM, 1);
  M5.Lcd.drawCentreString(btnName[btnNumber-1], BTNBXPOS + (BTNBW/2), M5.Lcd.height() - BTNYTEXTPOSFROMBOTTOM, 1);
  M5.Lcd.drawCentreString(">", BTNCXPOS + (BTNCW/2), M5.Lcd.height() - BTNYTEXTPOSFROMBOTTOM, 1);
}
//------------------------------------------------------------------------------
void drawBarGraph(bool active = true) { 
  uint16_t dig1=0;
  uint16_t dig2=0;
  uint16_t dig3=0;
  uint16_t dig4=0;
  uint16_t digall=0;

  if (active == false) {
    M5.Lcd.fillRect(BARGRAPHPOSX, BARGRAPHPOSY, TFT_HEIGHT - 70, 2, COLORNOTACTIVE); // bargraph bottom init scale
  } else {  
    if (valuechar[REGDIG1] >= 0x30 && valuechar[REGDIG1] <= 0x39)
      dig1 = (valuechar[REGDIG1] - 0x30) * 1000;
    if (valuechar[REGDIG2] >= 0x30 && valuechar[REGDIG2] <= 0x39)
      dig2 = (valuechar[REGDIG2] - 0x30) * 100;
    if (valuechar[REGDIG3] >= 0x30 && valuechar[REGDIG3] <= 0x39)
      dig3 = (valuechar[REGDIG3] - 0x30) * 10;
    if (valuechar[REGDIG4] >= 0x30 && valuechar[REGDIG4] <= 0x39)
      dig4 = (valuechar[REGDIG4] - 0x30) * 1;
    digall = dig1 + dig2 + dig3 + dig4;
  }

  uint16_t mapval = map(digall,0,6000,0,240);
  uint16_t CURCOL;

  for(uint16_t i = 0; i <= 240; i++) {
    if (i <= mapval && active == true)
      CURCOL = TFT_WHITE;
    else
      CURCOL = COLORNOTACTIVE;
    
    if (i%4==0) {
      if (i%5==0) {
        if (i%40==0)
          M5.Lcd.drawFastVLine(BARGRAPHADDX+i, BARGRAPHYA, 20, CURCOL);
        else
          M5.Lcd.drawFastVLine(BARGRAPHADDX+i, BARGRAPHYB, 15, CURCOL);
      } else {
        M5.Lcd.drawFastVLine(BARGRAPHADDX+i, BARGRAPHYC, 10, CURCOL);
      }  
    }
  } 
}
//------------------------------------------------------------------------------
void displayShow(bool active = false) {
  drawIcon(WBLEPOSX, TOPROWPOSY, ICONW, ICONH, BLE_BMP, deviceBleConnected == true?COLORICONBLE:COLORNOTACTIVE);
  drawIcon(WAUTOPOSX, TOPROWPOSY, ICONW*2, ICONH, AUTO_BMP, active == true?COLORICONAUTO:COLORNOTACTIVE);
  drawIcon(WMAXPOSX, TOPROWPOSY, ICONW*2, ICONH, MAX_BMP, active == true?COLORICONMAX:COLORNOTACTIVE);
  drawIcon(WMINPOSX, TOPROWPOSY, ICONW*2, ICONH, MIN_BMP, active == true?COLORICONMIN:COLORNOTACTIVE);
  drawIcon(WHOLDPOSX, TOPROWPOSY, ICONW, ICONH, HOLD_BMP, active == true?COLORICONHOLD:COLORNOTACTIVE);
  drawIcon(WRELPOSX, TOPROWPOSY, ICONW, ICONH, REL_BMP, active == true?COLORICONREL:COLORNOTACTIVE);
  drawIcon(WDIODEPOSX, TOPROWPOSY, ICONW, ICONH, DIODE_BMP, active == true?COLORICONDIODE:COLORNOTACTIVE);
  drawIcon(WBUZZPOSX, TOPROWPOSY, ICONW, ICONH, BUZZ_BMP, active == true?COLORICONBUZZ:COLORNOTACTIVE);
  drawIcon(WHVPOSX, TOPROWPOSY, ICONW, ICONH, HV_BMP, active == true?COLORICONHV:COLORNOTACTIVE);

  drawIcon(DCPOSX, DCPOSY, ICONW*2, ICONH, DC_BMP, active == true?COLORICONDC:COLORNOTACTIVE);
  drawIcon(ACPOSX, ACPOSY, ICONW*2, ICONH, AC_BMP, active == true?COLORICONAC:COLORNOTACTIVE);

  M5.Lcd.fillRect(SIGNPOSX, SIGNPOSY, SIGNW, SIGNH, active == true?FONTCOLORVALUE:COLORNOTACTIVE);

  M5.Lcd.fillRect(POINTSPOSX + (0 * DIGITSDISTANCE), POINTSPOSY, POINTSW, POINTSH, active == true?FONTCOLORVALUE:COLORNOTACTIVE);
  M5.Lcd.fillRect(POINTSPOSX + (1 * DIGITSDISTANCE), POINTSPOSY, POINTSW, POINTSH, active == true?FONTCOLORVALUE:COLORNOTACTIVE);
  M5.Lcd.fillRect(POINTSPOSX + (2 * DIGITSDISTANCE), POINTSPOSY, POINTSW, POINTSH, active == true?FONTCOLORVALUE:COLORNOTACTIVE);

  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(active == true?FONTCOLORVALUE:COLORNOTACTIVE, BACKGROUND);
  M5.Lcd.drawNumber(8, DIGITSPOSX + (0 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);
  M5.Lcd.drawNumber(8, DIGITSPOSX + (1 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);
  M5.Lcd.drawNumber(8, DIGITSPOSX + (2 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);
  M5.Lcd.drawNumber(8, DIGITSPOSX + (3 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);

  M5.Lcd.setTextSize(1);
  M5.Lcd.fillRect(SCALEPOSX, SCALEUNITPOSY, SCALEW, SCALEUNITH, BACKGROUND);
  M5.Lcd.fillRect(UNITPOSX, SCALEUNITPOSY, UNITW, SCALEUNITH, BACKGROUND);
  M5.Lcd.drawString("M", SCALEPOSX, SCALEUNITPOSY, SCALEUNITFONT);
  M5.Lcd.drawString("Ohm", UNITPOSX, SCALEUNITPOSY, SCALEUNITFONT);

  drawBarGraph(active);

  drawButtons();

  if (active == false)
    firstNotify = true;
}
//------------------------------------------------------------------------------
void parseMeterPlus() {
  //your code for parse meterPlus from rawvalbuf to b35t format and fill valuechar buffer
  //rawvalbuf e.x. 19 F0 04 00 49 04

  uint16_t temp16 = 0;
  uint8_t function, scale, decimal;
  float measurement;

  //prepare valuechar buff
  memset(valuechar, 0, meterReplySize);                                         // clean valuechar buffer
  valuechar[5]=0x20;
  valuechar[12]=0x0d;
  valuechar[13]=0x0a;

  //------------------------------------------------------------------
  //------------------------------------------------------------------

  // 0 and 1 byte from rawvalbuf
  temp16 = *((uint16_t *)(rawvalbuf));

  DEBUG_MSG("D: B35tPlus PAIR1: %04X\n", temp16);

  // parse
  function = (temp16 >> 6) & 0x0f;
  scale = (temp16 >> 3) & 0x07;
  decimal = temp16 & 0x07;

  // decode and set appropriate bits in valuechar
  DEBUG_MSG(" - FUNCT: %02d\n", function);
  DEBUG_MSG(" - SCALE: %02d\n", scale);
  DEBUG_MSG(" - DECIM: %02d\n", decimal);

  //decinal point
  switch (decimal) {
    case B000:
      valuechar[REGPOINT]=FLAGPOINT0;
      break;
    case B001:
      valuechar[REGPOINT]=FLAGPOINT1;
      break;
    case B010:
      valuechar[REGPOINT]=FLAGPOINT2;
      break;
    case B011:
      valuechar[REGPOINT]=FLAGPOINT3;
      break;
    default:
      break;
  }

  //function
  switch (function) {
    case B0000:                                         //DCV
      bitSet(valuechar[REGUNIT], 7);                    //V
      bitSet(valuechar[REGMODE], 4);                    //DC
      break;
    case B0001:                                         //ACV
      bitSet(valuechar[REGUNIT], 7);                    //V
      bitSet(valuechar[REGMODE], 3);                    //AC
      break;
    case B0010:                                         //DCA
      bitSet(valuechar[REGUNIT], 6);                    //A
      bitSet(valuechar[REGMODE], 4);                    //DC
      break;
    case B0011:                                         //ACA
      bitSet(valuechar[REGUNIT], 6);                    //A
      bitSet(valuechar[REGMODE], 3);                    //AC
      break;
    case B0100:                                         //Ohm
      bitSet(valuechar[REGUNIT], 5);                    //Ohm
      break;
    case B0101:                                         //Cap
      bitSet(valuechar[REGUNIT], 2);                    //Cap
      break;
    case B0110:                                         //Hz
      bitSet(valuechar[REGUNIT], 3);                    //Hz
      break;
    case B0111:                                         //Duty
      bitSet(valuechar[REGSCALE], 1);                   //Duty
      break;
    case B1000:                                         //TempC
      bitSet(valuechar[REGUNIT], 1);                    //TempC
      break;
    case B1001:                                         //TempF
      bitSet(valuechar[REGUNIT], 0);                    //TempF
      break;
    case B1010:                                         //Diode
      bitSet(valuechar[REGSCALE], 2);                   //Diode
      break;
    case B1011:                                         //Continuity
      bitSet(valuechar[REGSCALE], 3);                   //Continuity
      break;
    case B1100:                                         //hFE
      bitSet(valuechar[REGUNIT], 4);                    //hFE
      break;
    default:
      break;
  }

  //scale
  switch (scale) {
    case B010:                                          //micro
      bitSet(valuechar[REGSCALE], 7);                   //micro
      break;
    case B011:                                          //milli
      bitSet(valuechar[REGSCALE], 6);                   //milli
      break;
    case B101:                                          //kilo
      bitSet(valuechar[REGSCALE], 5);                   //kilo
      break;
    case B110:                                          //mega
      bitSet(valuechar[REGSCALE], 4);                   //mega
      break;
    default:
      break;
  }

  //------------------------------------------------------------------
  //------------------------------------------------------------------

  // 2 and 3 byte from rawvalbuf
  temp16 = *((uint16_t *)(rawvalbuf+2));

  DEBUG_MSG("D: B35tPlus PAIR2: %04X\n", temp16);

  // decode and set appropriate bits in valuechar
  bitWrite(valuechar[REGMODE], 1, bitRead(temp16, 0));    // HOLD
  bitWrite(valuechar[REGMODE], 2, bitRead(temp16, 1));    // DELTA
  bitWrite(valuechar[REGMODE], 5, bitRead(temp16, 2));    // AUTORANGE
  bitWrite(valuechar[REGMINMAX], 4, bitRead(temp16, 4));  // MIN
  bitWrite(valuechar[REGMINMAX], 5, bitRead(temp16, 5));  // MAX
  if (bitRead(temp16, 3) == 1) {                          // LOW BAT
    if (!meterPlusLowBat) {
      meterPlusLowBat = true;
      batCheckDraw();
    }
  } else {
    if (meterPlusLowBat) {
      meterPlusLowBat = false;
      batCheckDraw();
    }
  }

  //------------------------------------------------------------------
  //------------------------------------------------------------------

  // 4 and 5 byte from rawvalbuf
  temp16 = *((uint16_t *)(rawvalbuf+4));

  DEBUG_MSG("D: B35tPlus PAIR3: %04X\n", temp16);

  // parse
  if (temp16 < 0x7fff) {
    measurement = (float)temp16 / pow(10.0, decimal);
    valuechar[REGPLUSMINUS]=FLAGPLUS;                                          // set flag plus 
  } else {
    measurement = -1 * (float)(temp16 & 0x7fff) / pow(10.0, decimal);
    valuechar[REGPLUSMINUS]=FLAGMINUS;                                          // set flag minus
    temp16 = temp16 & 0x7fff;
  }

  valuechar[REGDIG1] = 0x30 + (int(temp16 / 1000) % 10);
  valuechar[REGDIG2] = 0x30 + (int(temp16 / 100) % 10);
  valuechar[REGDIG3] = 0x30 + (int(temp16 / 10) % 10);
  valuechar[REGDIG4] = 0x30 + (int(temp16 / 1) % 10);

  DEBUG_MSG(" - VALUE : %09.4f\n", measurement);
  DEBUG_MSG(" - DIGITS: %c %c %c %c\n", valuechar[REGDIG1], valuechar[REGDIG2], valuechar[REGDIG3], valuechar[REGDIG4]);

}
//------------------------------------------------------------------------------
void displayValues() {

  static char valuetmp[meterReplySize-3];                                       // local copy of data

  if (meterIsPlus) {                                                            // if meter is b35tPLUS
    parseMeterPlus();
  } else {                                                                      // if meter is b35t
    memcpy(valuechar, rawvalbuf, meterReplySize);                               // simply copy rawvalbuf to valuechar
  }

  if (firstNotify == true) {
    displayShow();
    M5.Lcd.setTextColor(FONTCOLORVALUE, BACKGROUND);
    M5.Lcd.setTextDatum(TL_DATUM);
    M5.Lcd.fillRect(BARGRAPHPOSX, BARGRAPHPOSY, TFT_HEIGHT - 70, 2, TFT_WHITE);  // bargraph bottom init scale
    memset(valuetmp, 0, meterReplySize-3);
    //drawIcon(WBLEPOSX, TOPROWPOSY, ICONW, ICONH, BLE_BMP, deviceBleConnected == true?COLORICONBLE:COLORNOTACTIVE);
    firstNotify = false;
  } else {
    drawBarGraph();
  }  

  if (valuetmp[REGUNIT] != valuechar[REGUNIT]) {                                // unit
    String unit = "";
    valuetmp[REGUNIT] = valuechar[REGUNIT];
    switch (valuetmp[REGUNIT]) {
      case FLAGUNITFAHR:
        unit = "*F";
        break;
      case FLAGUNITGRAD:
        unit = "*C";
        break;
      case FLAGUNITNF:
        unit = "nF";
        break;
      case FLAGUNITHZ:
        unit = "Hz";
        break;
      case FLAGUNITHFE:
        unit = "hFE";
        break;
      case FLAGUNITOHM:
        unit = "Ohm";
        break;
      case FLAGUNITAMP:
        unit = "A";
        break;
      case FLAGUNITVOLT:
        unit = "V";
        break;
      break;
    }
    M5.Lcd.fillRect(UNITPOSX, SCALEUNITPOSY, UNITW, SCALEUNITH, BACKGROUND);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_YELLOW, BACKGROUND);
    M5.Lcd.drawString(unit, UNITPOSX, SCALEUNITPOSY, SCALEUNITFONT);      
  }

  if (valuetmp[REGSCALE] != valuechar[REGSCALE]) {                              //scale
    String scale = "";
    if ((valuechar[REGSCALE] & FLAGSCALEDUTY) == FLAGSCALEDUTY) {
      valuetmp[REGSCALE] |= FLAGSCALEDUTY;
      scale = "%";  
    } else {
      valuetmp[REGSCALE] &= ~(FLAGSCALEDUTY);  
    }

    if ((valuechar[REGSCALE] & FLAGSCALEDIODE) == FLAGSCALEDIODE)
      valuetmp[REGSCALE] |= FLAGSCALEDIODE;
    else
      valuetmp[REGSCALE] &= ~(FLAGSCALEDIODE);

    drawIcon(WDIODEPOSX, TOPROWPOSY, ICONW, ICONH, DIODE_BMP, (valuetmp[REGSCALE] & FLAGSCALEDIODE) == FLAGSCALEDIODE?COLORICONDIODE:COLORNOTACTIVE);

    if ((valuechar[REGSCALE] & FLAGSCALEBUZZ) == FLAGSCALEBUZZ) {
      valuetmp[REGSCALE] |= FLAGSCALEBUZZ;
    } else {
      valuetmp[REGSCALE] &= ~(FLAGSCALEBUZZ);
    }

    drawIcon(WBUZZPOSX, TOPROWPOSY, ICONW, ICONH, BUZZ_BMP, (valuetmp[REGSCALE] & FLAGSCALEBUZZ) == FLAGSCALEBUZZ?COLORICONBUZZ:COLORNOTACTIVE);

    if ((valuechar[REGSCALE] & FLAGSCALEMEGA) == FLAGSCALEMEGA) {
      valuetmp[REGSCALE] |= FLAGSCALEMEGA;
      scale = "M";  
    } else {
      valuetmp[REGSCALE] &= ~(FLAGSCALEMEGA);  
    }

    if ((valuechar[REGSCALE] & FLAGSCALEKILO) == FLAGSCALEKILO) {
      valuetmp[REGSCALE] |= FLAGSCALEKILO;
      scale = "k";  
    } else {
      valuetmp[REGSCALE] &= ~(FLAGSCALEKILO);  
    }

    if ((valuechar[REGSCALE] & FLAGSCALEMILI) == FLAGSCALEMILI) {
      valuetmp[REGSCALE] |= FLAGSCALEMILI;
      scale = "m";  
    } else {
      valuetmp[REGSCALE] &= ~(FLAGSCALEMILI);  
    }

    if ((valuechar[REGSCALE] & FLAGSCALEMICRO) == FLAGSCALEMICRO) {
      valuetmp[REGSCALE] |= FLAGSCALEMICRO;
      scale = "u";  
    } else {
      valuetmp[REGSCALE] &= ~(FLAGSCALEMICRO);  
    }

    M5.Lcd.fillRect(SCALEPOSX, SCALEUNITPOSY, SCALEW, SCALEUNITH, BACKGROUND);
    if (scale != "") {
      M5.Lcd.setTextSize(1);
      M5.Lcd.setTextColor(TFT_YELLOW, BACKGROUND);
      M5.Lcd.drawString(scale, SCALEPOSX, SCALEUNITPOSY, SCALEUNITFONT);
    } 
  }

  if ((valuetmp[REGMODE] & FLAGMODEAUTO) != (valuechar[REGMODE] & FLAGMODEAUTO)) {  // auto
    if ((valuechar[REGMODE] & FLAGMODEAUTO) == FLAGMODEAUTO)
      valuetmp[REGMODE] |= FLAGMODEAUTO;
    else
      valuetmp[REGMODE] &= ~(FLAGMODEAUTO);

    drawIcon(WAUTOPOSX, TOPROWPOSY, ICONW*2, ICONH, AUTO_BMP, (valuetmp[REGMODE] & FLAGMODEAUTO) == FLAGMODEAUTO?COLORICONAUTO:COLORNOTACTIVE);

  }

  if ((valuetmp[REGMINMAX] & FLAGMAX) != (valuechar[REGMINMAX] & FLAGMAX)) {    // max
    if ((valuechar[REGMINMAX] & FLAGMAX) == FLAGMAX)
      valuetmp[REGMINMAX] |= FLAGMAX;  
    else
      valuetmp[REGMINMAX] &= ~(FLAGMAX); 

    drawIcon(WMAXPOSX, TOPROWPOSY, ICONW*2, ICONH, MAX_BMP, (valuetmp[REGMINMAX] & FLAGMAX) == FLAGMAX?COLORICONMAX:COLORNOTACTIVE);

  }

  if ((valuetmp[REGMINMAX] & FLAGMIN) != (valuechar[REGMINMAX] & FLAGMIN)) {    // min
    if ((valuechar[REGMINMAX] & FLAGMIN) == FLAGMIN)
      valuetmp[REGMINMAX] |= FLAGMIN;  
    else
      valuetmp[REGMINMAX] &= ~(FLAGMIN);

    drawIcon(WMINPOSX, TOPROWPOSY, ICONW*2, ICONH, MIN_BMP, (valuetmp[REGMINMAX] & FLAGMIN) == FLAGMIN?COLORICONMIN:COLORNOTACTIVE);

  }

  if ((valuetmp[REGMODE] & FLAGMODEHOLD) != (valuechar[REGMODE] & FLAGMODEHOLD)) {  // hold
    if ((valuechar[REGMODE] & FLAGMODEHOLD) == FLAGMODEHOLD)
      valuetmp[REGMODE] |= FLAGMODEHOLD;
    else
      valuetmp[REGMODE] &= ~(FLAGMODEHOLD);

    drawIcon(WHOLDPOSX, TOPROWPOSY, ICONW, ICONH, HOLD_BMP, (valuetmp[REGMODE] & FLAGMODEHOLD) == FLAGMODEHOLD?COLORICONHOLD:COLORNOTACTIVE);

  }

  if ((valuetmp[REGMODE] & FLAGMODEREL) != (valuechar[REGMODE] & FLAGMODEREL)) {  // relative
    if ((valuechar[REGMODE] & FLAGMODEREL) == FLAGMODEREL)
      valuetmp[REGMODE] |= FLAGMODEREL;
    else
      valuetmp[REGMODE] &= ~(FLAGMODEREL);

    drawIcon(WRELPOSX, TOPROWPOSY, ICONW, ICONH, REL_BMP, (valuetmp[REGMODE] & FLAGMODEREL) == FLAGMODEREL?COLORICONREL:COLORNOTACTIVE);

  }

  if ((valuetmp[REGMODE] & FLAGMODEDC) != (valuechar[REGMODE] & FLAGMODEDC)) {  // dc
    if ((valuechar[REGMODE] & FLAGMODEDC) == FLAGMODEDC)
      valuetmp[REGMODE] |= FLAGMODEDC;  
    else
      valuetmp[REGMODE] &= ~(FLAGMODEDC);

    drawIcon(DCPOSX, DCPOSY, ICONW*2, ICONH, DC_BMP, (valuetmp[REGMODE] & FLAGMODEDC) == FLAGMODEDC?COLORICONDC:COLORNOTACTIVE);

  }

  if ((valuetmp[REGMODE] & FLAGMODEAC) != (valuechar[REGMODE] & FLAGMODEAC)) {  // ac
    if ((valuechar[REGMODE] & FLAGMODEAC) == FLAGMODEAC)
      valuetmp[REGMODE] |= FLAGMODEAC;   
    else
      valuetmp[REGMODE] &= ~(FLAGMODEAC);

    drawIcon(ACPOSX, ACPOSY, ICONW*2, ICONH, AC_BMP, (valuetmp[REGMODE] & FLAGMODEAC) == FLAGMODEAC?COLORICONAC:COLORNOTACTIVE);

  }

  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(FONTCOLORVALUE, BACKGROUND);

  if (valuetmp[REGPLUSMINUS] != valuechar[REGPLUSMINUS]) {                      // sign "-"
    valuetmp[REGPLUSMINUS] = valuechar[REGPLUSMINUS];
    M5.Lcd.fillRect(SIGNPOSX, SIGNPOSY, SIGNW, SIGNH, (valuetmp[REGPLUSMINUS] & FLAGMINUS) == FLAGMINUS?FONTCOLORVALUE:BACKGROUND);
  }

  if (valuetmp[REGDIG1] != valuechar[REGDIG1]) {                                // digits
    valuetmp[REGDIG1] = valuechar[REGDIG1];
    if (valuetmp[REGDIG1] >= 0x30 && valuetmp[REGDIG1] <= 0x39)
      M5.Lcd.drawChar(valuetmp[REGDIG1], DIGITSPOSX + (0 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);
    else if (valuetmp[REGDIG1] == 0x3a)
      M5.Lcd.drawNumber(1, DIGITSPOSX + (0 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);
    else
      M5.Lcd.fillRect(DIGITSPOSX + (0 * DIGITSDISTANCE), DIGITSPOSY, 63, 94, BACKGROUND);  
  }

  if (valuetmp[REGDIG2] != valuechar[REGDIG2]) {
    valuetmp[REGDIG2] = valuechar[REGDIG2];
    if (valuetmp[REGDIG2] >= 0x30 && valuetmp[REGDIG2] <= 0x39) {
      M5.Lcd.drawChar(valuetmp[REGDIG2], DIGITSPOSX + (1 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);
    } else if (valuetmp[REGDIG2] == 0x3a) {
      M5.Lcd.drawNumber(1, DIGITSPOSX + (1 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);
    } else {
      M5.Lcd.fillRect(DIGITSPOSX + (1 * DIGITSDISTANCE), DIGITSPOSY, 63, 94, BACKGROUND);
    } 
  }

  if (valuetmp[REGDIG3] != valuechar[REGDIG3]) {
    valuetmp[REGDIG3] = valuechar[REGDIG3];
    if (valuetmp[REGDIG3] >= 0x30 && valuetmp[REGDIG3] <= 0x39)
      M5.Lcd.drawChar(valuetmp[REGDIG3], DIGITSPOSX + (2 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);
    else if (valuetmp[REGDIG3] == 0x3a)
      M5.Lcd.drawNumber(1, DIGITSPOSX + (2 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);
    else
      M5.Lcd.fillRect(DIGITSPOSX + (2 * DIGITSDISTANCE), DIGITSPOSY, 63, 94, BACKGROUND); 
  }

  if (valuetmp[REGDIG4] != valuechar[REGDIG4]) {
    valuetmp[REGDIG4] = valuechar[REGDIG4];
    if (valuetmp[REGDIG4] >= 0x30 && valuetmp[REGDIG4] <= 0x39)
      M5.Lcd.drawChar(valuetmp[REGDIG4], DIGITSPOSX + (3 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);
    else if (valuetmp[REGDIG4] == 0x3a)
      M5.Lcd.drawNumber(1, DIGITSPOSX + (3 * DIGITSDISTANCE), DIGITSPOSY, DIGITSFONT);
    else
      M5.Lcd.fillRect(DIGITSPOSX + (3 * DIGITSDISTANCE), DIGITSPOSY, 63, 94, BACKGROUND); 
  }

  if (valuetmp[REGPOINT] != valuechar[REGPOINT]) {                              // decimal point
    valuetmp[REGPOINT] = valuechar[REGPOINT];
    M5.Lcd.fillRect(POINTSPOSX + (0 * DIGITSDISTANCE), POINTSPOSY, POINTSW, POINTSH, (valuetmp[REGPOINT] & FLAGPOINT1) == FLAGPOINT1?FONTCOLORVALUE:BACKGROUND);
    M5.Lcd.fillRect(POINTSPOSX + (1 * DIGITSDISTANCE), POINTSPOSY, POINTSW, POINTSH, (valuetmp[REGPOINT] & FLAGPOINT2) == FLAGPOINT2?FONTCOLORVALUE:BACKGROUND);
    M5.Lcd.fillRect(POINTSPOSX + (2 * DIGITSDISTANCE), POINTSPOSY, POINTSW, POINTSH, (valuetmp[REGPOINT] & FLAGPOINT3) == FLAGPOINT3?FONTCOLORVALUE:BACKGROUND);
  }

  newBleData = false;

}

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
#ifndef FALSEMETER
  if (isNotify == true && length <= rawBufferSize && pBLERemoteCharacteristic->getUUID().equals(charnotificationUUID)) {

    //DEBUG_MSG("I: Notify callback len=%d (UUID: %s)\n", length, pBLERemoteCharacteristic->getUUID().toString().c_str());
    DEBUG_MSG("I: Notify callback len=%d (UUID OK)\n", length);

    if (memcmp(rawvalbuf, pData, length) != 0) {                                // if new data <> old data
      if (newBleData == false) {                                                // and if old data are displayed then copy new data
        memcpy(rawvalbuf, pData, length);

#ifdef MYDEBUG
        DEBUG_MSG("D: RAW BUFF: [ ");
        for (uint8_t i = 0; i < length; i++) {
          DEBUG_MSG("%02X ", rawvalbuf[i]);
        }
        DEBUG_MSG("]\n");
#endif

        if ((length == replySizePlus) && (rawvalbuf[1] >= 0xf0)) {              // if meter PLUS detected
          if (meterIsPlus == false) {
            meterIsPlus = true;
            DEBUG_MSG("D: Type B35TPlus\n");
          }
          newBleData = true;
        } else if ((length == replySizeNorm) && (rawvalbuf[12] == 0x0d) && (rawvalbuf[13] == 0x0a)) {
          if (meterIsPlus == true) {
            meterIsPlus = false;
            DEBUG_MSG("D: Type B35T\n");
          }
          newBleData = true;
        }

      }
    }
    lastBleNotify = millis();
  }
#endif
}

#ifdef FALSEMETER
static void notifyTest() {
  uint8_t _pData[replySizePlus];
  _pData[0] = 0x22;
  _pData[1] = 0xf0;
  _pData[2] = 0x04;
  _pData[3] = 0x00;
  _pData[4] = 0xe7;
  _pData[5] = 0x03;


    if (memcmp(rawvalbuf, _pData, replySizePlus) != 0) {                                // if new data <> old data
      if (newBleData == false) {                                                // and if old data are displayed then copy new data
        memcpy(rawvalbuf, _pData, replySizePlus);

#ifdef MYDEBUG
        DEBUG_MSG("D: RAW BUFF: [ ");
        for (uint8_t i = 0; i < replySizePlus; i++) {
          DEBUG_MSG("%02X ", rawvalbuf[i]);
        }
        DEBUG_MSG("]\n");
#endif

          if (meterIsPlus == false) {
            meterIsPlus = true;
          }
      }
      newBleData = true;
    }
    lastBleNotify = millis();

}
#endif

class MyClientCallbacks: public BLEClientCallbacks {
  void onConnect(BLEClient *pClient) {
    deviceBleConnected = true;                                                  // set ble connected flag
    DEBUG_MSG("I: %s connected\n", OWONNAME);
  };

  void onDisconnect(BLEClient *pClient) {
    pClient->disconnect();
    deviceBleConnected = false;                                                 // clear ble connected flag
    deviceBleWriteAvailable = false;                                            // clear ble available for write flag
    DEBUG_MSG("I: %s disconnected\n", OWONNAME);
  }
};

bool connectToServer(BLEAddress pAddress) {

  DEBUG_MSG("I: Create a connection to addr: %s\n", pAddress.toString().c_str());

  BLEClient*  pClient  = BLEDevice::createClient();

  DEBUG_MSG(" - Client created\n");

  pClient->setClientCallbacks(new MyClientCallbacks());

  DEBUG_MSG(" - Connecting to server...\n");

  pClient->connect(pAddress);                                                   // connect to the remove BLE Server.

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);          // check if remote BLE service exists
  if (pRemoteService == nullptr) {      
    DEBUG_MSG(" - Service not found (UUID: %s)\n", serviceUUID.toString().c_str());
    return false;
  } else {
    DEBUG_MSG(" - Service found (UUID: %s)\n", serviceUUID.toString().c_str());
  }

  // notify characteristic
  pRemoteCharacteristicNotify = pRemoteService->getCharacteristic(charnotificationUUID);
  if (pRemoteCharacteristicNotify == nullptr) {
    DEBUG_MSG(" - Notify characteristic not found (UUID: %s)\n", charnotificationUUID.toString().c_str());
    return false;
  } else {
    DEBUG_MSG(" - Notify characteristic found (UUID: %s)\n", charnotificationUUID.toString().c_str());
  }
  pRemoteCharacteristicNotify->registerForNotify(notifyCallback);               //register notify callback

  // write characteristic
  pRemoteCharacteristicWrite = pRemoteService->getCharacteristic(charwriteUUID);
  if (pRemoteCharacteristicWrite == nullptr) {
    DEBUG_MSG(" - Write characteristic not found (UUID: %s)\n", charwriteUUID.toString().c_str());
    deviceBleWriteAvailable = false;                                            // clear ble available for write flag
  } else {
    DEBUG_MSG(" - Write characteristic found (UUID: %s)\n", charwriteUUID.toString().c_str());
    deviceBleWriteAvailable = true;                                             // set ble available for write flag
  }

  return true;

}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {

  void onResult(BLEAdvertisedDevice advertisedDevice) {
    DEBUG_MSG("I: BLE scan stop\n");
    if (advertisedDevice.haveName() && strcmp(advertisedDevice.getName().c_str(), OWONNAME) == 0) {
      advertisedDevice.getScan()->stop();
      //DEBUG_MSG("I: Stop BLE scan\n");
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      DEBUG_MSG("I: BLE device found (%s at addr: %s)\n", OWONNAME, pServerAddress->toString().c_str());
    } else {
      DEBUG_MSG("I: BLE device not found\n");
    }
    drawIcon(WBLEPOSX, TOPROWPOSY, ICONW, ICONH, BLE_BMP, deviceBleConnected == true?COLORICONBLE:COLORNOTACTIVE);
  }

};

void doScan() {
  drawIcon(WBLEPOSX, TOPROWPOSY, ICONW, ICONH, BLE_BMP, COLORICONBLESEARCH);
  DEBUG_MSG("I: BLE scan start\n");
  startBleScanning = millis();
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(scanTime);
}

void setup() {
  Serial.begin(115200);
  DEBUG_MSG("I: Start OWON B35T Client\n");

  Wire.begin();
  //batMeterInit();                                                             // init MAX17043

  WiFi.persistent(false);
  WiFi.enableSTA(false);
  WiFi.enableAP(false);
  WiFi.mode(WIFI_OFF);

  M5.begin();
  if(digitalRead(BUTTON_A_PIN) == 0) {
    Serial.println("Will Load menu binary");
    updateFromFS(SD);
    ESP.restart();
  }
  M5.Lcd.fillScreen(BACKGROUND);

  memset(rawvalbuf, 0, rawBufferSize);                                          // clean buffer
  memset(valuechar, 0, meterReplySize);                                         // clean buffer

  displayShow();

  batCheckDraw();

  BLEDevice::init("");
}

void loop() {

  M5.update();

  if (deviceBleConnected == false) {

    delay(250);

    if (startBleScanning != 0 && millis() > (startBleScanning + (scanTime*1000))) {
      startBleScanning = 0;
      return; 
    }

    if (startBleScanning == 0) {
      if (firstNotify == false)
        displayShow();
      else
        drawIcon(WBLEPOSX, TOPROWPOSY, ICONW, ICONH, BLE_BMP, deviceBleConnected == true?COLORICONBLE:COLORNOTACTIVE);
      doScan();
      return; 
    }

    startBleScanning = 0;
    if (connectToServer(*pServerAddress)) {
      lastBleNotify = millis();
    } 
      
  } else {

    if (millis() > (lastBleNotify + maxWaitForNotify) && firstNotify == false) {
      DEBUG_MSG("I: No notify from %s (>%d)\n", OWONNAME, maxWaitForNotify);
      deviceBleWriteAvailable = false;                                          // clear ble available for write flag
      displayShow(); 
      return;
    }
    
    if (newBleData == true) {
      if (deviceBleWriteAvailable == false) {
        deviceBleWriteAvailable = true;                                         // set ble available for write flag
      }
      displayValues();
    } else

      drawIcon(WBLEPOSX, TOPROWPOSY, ICONW, ICONH, BLE_BMP, deviceBleConnected == true?COLORICONBLE:COLORNOTACTIVE);

    if (deviceBleWriteAvailable == true) {

      if (M5.BtnA.wasReleased()) {
        if (btnNumber > 1) {
          btnNumber--;
          owonBtnArray[0] = btnNumber;
          drawButtons();
          DEBUG_MSG("I: BTN set [%s]\n", btnName[btnNumber-1]);
        }
      }

      if (M5.BtnB.wasReleased()) {
        DEBUG_MSG("I: BTN short press [%s]\n", btnName[owonBtnArray[0]-1]); 
        owonBtnArray[1] = btnShortPress;     
        pRemoteCharacteristicWrite->writeValue(owonBtnArray, owonBtnArraySize);
      } else if (M5.BtnB.pressedFor(btnLongPressTime)) {
        DEBUG_MSG("I: BTN long  press [%s]\n", btnName[owonBtnArray[0]-1]);
        if (owonBtnArray[0] == 1 || owonBtnArray[0] == 5)                       // select and hz/duty accept only short press
          owonBtnArray[1] = btnShortPress;
        else
          owonBtnArray[1] = btnLongPress;     
        pRemoteCharacteristicWrite->writeValue(owonBtnArray, owonBtnArraySize);
        while(M5.BtnB.isPressed()){
          M5.update();
        }        
      }

      if (M5.BtnC.wasReleased()) {
        if (btnNumber < buttonsMax) {
          btnNumber++;
          owonBtnArray[0] = btnNumber;
          drawButtons();
          DEBUG_MSG("I: BTN set [%s]\n", btnName[btnNumber-1]);
        }
      }

    }

  }

#ifdef BATMETERI2C
  if (millis() > batNextReadTime) {
    batCheckDraw();
    batNextReadTime = millis() + batReadEvery;
  }
#endif

#ifdef FALSEMETER
  if (millis() > batNextReadTime) {
    notifyTest();
    batNextReadTime = millis() + 2000;
  }
#endif

}
