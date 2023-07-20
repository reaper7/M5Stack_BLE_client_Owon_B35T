/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * <Reaper7> wrote this code. As long as you retain this 
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
*/

// this is a part of
// M5Stack_BLE_client_Owon_B35T.ino
// 2023 Reaper7 (tested on M5Stack)

#ifndef METER_GRAPHICS_H
#define METER_GRAPHICS_H

//global color settings
const uint16_t BACKGROUND = TFT_BLACK;
const uint16_t FONTCOLORVALUE = TFT_LIGHTGREY;
//elements color
const uint16_t COLORNOTACTIVE = 0x2965;                                         // r,g,b = 45,45,45
const uint16_t COLORICONACCU = TFT_GREEN;
const uint16_t COLORICONBLE = TFT_BLUE;
const uint16_t COLORICONBLESEARCH = TFT_YELLOW;
const uint16_t COLORICONAUTO = TFT_LIGHTGREY;
const uint16_t COLORICONMAX = TFT_RED;
const uint16_t COLORICONMIN = TFT_GREEN;
const uint16_t COLORICONREL = TFT_OLIVE;
const uint16_t COLORICONDIODE = TFT_MAGENTA;
const uint16_t COLORICONBUZZ = TFT_ORANGE;
const uint16_t COLORICONHV = TFT_GREENYELLOW;
const uint16_t COLORICONHOLD = TFT_BLUE;
const uint16_t COLORICONBAT = TFT_RED;
const uint16_t COLORICONDC = TFT_CYAN;
const uint16_t COLORICONAC = TFT_MAGENTA;
//elements position & size
//digits
const uint16_t DIGITSFONT = 7;
const uint16_t DIGITSPOSY = 35;
const uint16_t DIGITSPOSX = 40;
const uint16_t DIGITSDISTANCE = 73;
//decimal points
const uint16_t POINTSW = 8;
const uint16_t POINTSH = 10;
const uint16_t POINTSPOSY = DIGITSPOSY + 84;
const uint16_t POINTSPOSX = DIGITSPOSX + 64;
//sign "-"
const uint16_t SIGNW = 30;
const uint16_t SIGNH = 10;
const uint16_t SIGNPOSY = DIGITSPOSY + 42;
const uint16_t SIGNPOSX = DIGITSPOSX - (SIGNW + 3);
//ac/dc
const uint16_t DCPOSY = SIGNPOSY + SIGNH + 5;
const uint16_t ACPOSY = DCPOSY + 21;
const uint16_t DCPOSX = DIGITSPOSX - 34;
const uint16_t ACPOSX = DIGITSPOSX - 34;
//scale
const uint16_t SCALEUNITFONT = 4;
const uint16_t SCALEUNITPOSY = DIGITSPOSY + 105;
const uint16_t SCALEPOSX = 235;
const uint16_t UNITPOSX = SCALEPOSX + 23;
const uint16_t SCALEW = 23;
const uint16_t UNITW = 60;   
const uint16_t SCALEUNITH = 20;
//bargraph
const uint16_t BARGRAPHPOSX = 35;
const uint16_t BARGRAPHPOSY = 185;

const uint16_t BARGRAPHADDX = BARGRAPHPOSX + 5;
const uint16_t BARGRAPHYA = BARGRAPHPOSY - 20;
const uint16_t BARGRAPHYB = BARGRAPHPOSY - 15;
const uint16_t BARGRAPHYC = BARGRAPHPOSY - 10;
//icons
const uint16_t ICONW = 16;
const uint16_t ICONH = 16;
const uint16_t ICONSMARGIN = 10;

const uint16_t TOPROWPOSX = 12;
const uint16_t TOPROWPOSY = 8;

const uint16_t WACCUPOSX = TOPROWPOSX;
const uint16_t WBLEPOSX = WACCUPOSX + ICONW + ICONSMARGIN;
const uint16_t WAUTOPOSX = WBLEPOSX + ICONW + ICONSMARGIN;
const uint16_t WMAXPOSX = WAUTOPOSX + (ICONW * 2) + ICONSMARGIN;
const uint16_t WMINPOSX = WMAXPOSX + (ICONW * 2) + ICONSMARGIN;
const uint16_t WHOLDPOSX = WMINPOSX + (ICONW * 2) + ICONSMARGIN;
const uint16_t WRELPOSX = WHOLDPOSX + ICONW + ICONSMARGIN;
const uint16_t WDIODEPOSX = WRELPOSX + ICONW + ICONSMARGIN;
const uint16_t WBUZZPOSX = WDIODEPOSX + ICONW + ICONSMARGIN;
const uint16_t WHVPOSX = WBUZZPOSX + ICONW + ICONSMARGIN;

const uint16_t ACCUSCALW = 2;
const uint16_t ACCUSCALH = 6;

//buttons
const uint16_t BTNAXPOS = 34;
const uint16_t BTNBXPOS = 108;
const uint16_t BTNCXPOS = 242;

const uint16_t BTNYPOSFROMBOTTOM = 28;

const uint16_t BTNAW = 40;
const uint16_t BTNBW = 100;
const uint16_t BTNCW = 40;

const uint16_t BTNH = 28;

const uint16_t BTNYTEXTPOSFROMBOTTOM = 21;

const uint8_t PROGMEM ACCU_BMP[32] = {
  B00000000,	B00000000,
  B00000000,	B00000000,
  B00000000,	B00000000,
  B11111111,	B11111110,
  B10000000,	B00000010,
  B10000000,	B00000011,
  B10000000,	B00000011,
  B10000000,	B00000011,
  B10000000,	B00000011,
  B10000000,	B00000011,
  B10000000,	B00000011,
  B10000000,	B00000010,
  B11111111,	B11111110,
  B00000000,	B00000000,
  B00000000,	B00000000,
  B00000000,	B00000000
};

const uint8_t PROGMEM BLE_BMP[32] = {
  B00000001,	B10000000,
  B00000001,	B11000000,
  B00010001,	B01100000,
  B00011001,	B00110000,
  B00001101,	B00011000,
  B00000111,	B00110000,
  B00000011,	B01100000,
  B00000001,	B11000000,
  B00000001,	B11000000,
  B00000011,	B01100000,
  B00000111,	B00110000,
  B00001101,	B00011000,
  B00011001,	B00110000,
  B00010001,	B01100000,
  B00000001,	B11000000,
  B00000001,	B10000000
};

const uint8_t PROGMEM AUTO_BMP[64] = {
  B01111100,	B01100011,	B01111111,	B00111110,
  B11111110,	B01100011,	B01111111,	B01111111,
  B11101110,	B01100011,	B00011100,	B01110111,
  B11000110,	B01100011,	B00011100,	B01100011,
  B11000110,	B01100011,	B00011100,	B01100011,
  B11000110,	B01100011,	B00011100,	B01100011,
  B11000110,	B01100011,	B00011100,	B01100011,
  B11111110,	B01100011,	B00011100,	B01100011,
  B11111110,	B01100011,	B00011100,	B01100011,
  B11000110,	B01100011,	B00011100,	B01100011,
  B11000110,	B01100011,	B00011100,	B01100011,
  B11000110,	B01100011,	B00011100,	B01100011,
  B11000110,	B01100011,	B00011100,	B01100011,
  B11000110,	B01110111,	B00011100,	B01110111,
  B11000110,	B00111110,	B00011100,	B01111111,
  B11000110,	B00111110,	B00011100,	B00111110
};

const uint8_t PROGMEM MAX_BMP[64] = {
  B11000000,	B01100011,	B11100011,	B00000011,
  B11100000,	B11100111,	B11110011,	B00000011,
  B11110001,	B11100111,	B01110011,	B10000111,
  B11111011,	B11100110,	B00110001,	B10000110,
  B11011111,	B01100110,	B00110001,	B11001110,
  B11001110,	B01100110,	B00110000,	B11001100,
  B11000100,	B01100110,	B00110000,	B11001100,
  B11000000,	B01100111,	B11110000,	B01111000,
  B11000000,	B01100111,	B11110000,	B01111000,
  B11000000,	B01100110,	B00110000,	B11001100,
  B11000000,	B01100110,	B00110000,	B11001100,
  B11000000,	B01100110,	B00110001,	B11001110,
  B11000000,	B01100110,	B00110001,	B10000110,
  B11000000,	B01100110,	B00110011,	B10000111,
  B11000000,	B01100110,	B00110011,	B00000011,
  B11000000,	B01100110,	B00110011,	B00000011
};

const uint8_t PROGMEM MIN_BMP[64] = {
  B11000000,	B01100001,	B10000110,	B00000011,
  B11100000,	B11100001,	B10000111,	B00000011,
  B11110001,	B11100001,	B10000111,	B10000011,
  B11111011,	B11100001,	B10000111,	B11000011,
  B11011111,	B01100001,	B10000110,	B11100011,
  B11001110,	B01100001,	B10000110,	B01110011,
  B11000100,	B01100001,	B10000110,	B00111011,
  B11000000,	B01100001,	B10000110,	B00011111,
  B11000000,	B01100001,	B10000110,	B00001111,
  B11000000,	B01100001,	B10000110,	B00000111,
  B11000000,	B01100001,	B10000110,	B00000011,
  B11000000,	B01100001,	B10000110,	B00000011,
  B11000000,	B01100001,	B10000110,	B00000011,
  B11000000,	B01100001,	B10000110,	B00000011,
  B11000000,	B01100001,	B10000110,	B00000011,
  B11000000,	B01100001,	B10000110,	B00000011
};

const uint8_t PROGMEM HOLD_BMP[32] = {
  B01111111,  B11111110,
  B11111111,  B11111111,
  B11100011,  B11000111,
  B11100011,  B11000111,
  B11100011,  B11000111,
  B11100011,  B11000111,
  B11100011,  B11000111,
  B11100000,  B00000111,
  B11100000,  B00000111,
  B11100011,  B11000111,
  B11100011,  B11000111,
  B11100011,  B11000111,
  B11100011,  B11000111,
  B11100011,  B11000111,
  B11111111,  B11111111,
  B01111111,  B11111110
};

const uint8_t PROGMEM REL_BMP[32] = {
  B00000001,  B00000000,
  B00000001,  B00000000,
  B00000011,  B10000000,
  B00000011,  B10000000,
  B00000110,  B11000000,
  B00000110,  B11000000,
  B00001100,  B01100000,
  B00001100,  B01100000,
  B00011000,  B00110000,
  B00011000,  B00110000,
  B00110000,  B00011000,
  B00110000,  B00011000,
  B01100000,  B00001100,
  B01100000,  B00001100,
  B11111111,  B11111110,
  B11111111,  B11111110
};

const uint8_t PROGMEM DIODE_BMP[32] = {
  B00001000,	B00011000,
  B00001100,	B00011000,
  B00001110,	B00011000,
  B00001111,	B00011000,
  B00001111,	B10011000,
  B00001111,	B11011000,
  B00001111,	B11111000,
  B11111111,	B11111111,
  B11111111,	B11111111,
  B00001111,	B11111000,
  B00001111,	B11011000,
  B00001111,	B10011000,
  B00001111,	B00011000,
  B00001110,	B00011000,
  B00001100,	B00011000,
  B00001000,	B00011000
};

const uint8_t PROGMEM BUZZ_BMP[32] = {
  B00000000,	B11000000,
  B00000001,	B11000000,
  B00000011,	B11000001,
  B00000111,	B11000001,
  B00001111,	B11000101,
  B11111111,	B11000101,
  B11111111,	B11010101,
  B11111111,	B11010101,
  B11111111,	B11010101,
  B11111111,	B11010101,
  B11111111,	B11000101,
  B00001111,	B11000101,
  B00000111,	B11000001,
  B00000011,	B11000001,
  B00000001,	B11000000,
  B00000000,	B11000000
};

const uint8_t PROGMEM HV_BMP[32] = {
  B00000000,	B10000000,
  B00000000,	B10000000,
  B00000001,	B10000000,
  B00000011,	B00000000,
  B00000111,	B00000000,
  B00000110,	B00010000,
  B00001100,	B01110000,
  B00001111,	B11100000,
  B00001111,	B11100000,
  B00011100,	B01100000,
  B00010000,	B11000000,
  B00000001,	B11000000,
  B00000001,	B10000000,
  B00000011,	B00000000,
  B00000010,	B00000000,
  B00000010,	B00000000
};

const uint8_t PROGMEM BAT_BMP[32] = {
  B00000000,	B00000000,
  B00000000,	B00000000,
  B00111000,	B00011100,
  B00111000,	B00011100,
  B11111111,	B11111111,
  B10000000,	B00000001,
  B10010000,	B00000001,
  B10111000,	B00011101,
  B10010000,	B00000001,
  B10000000,	B00000001,
  B10000000,	B00000001,
  B10000000,	B00000001,
  B10000000,	B00000001,
  B11111111,	B11111111
};

const uint8_t PROGMEM DC_BMP[64] = {
  B00111111,  B11000000,  B00000011,  B11000000,
  B00111111,  B11110000,  B00001111,  B11110000,
  B00111111,  B11111000,  B00011111,  B11111000,
  B00111000,  B00111000,  B00011100,  B00111100,
  B00111000,  B00011100,  B00111000,  B00011100,
  B00111000,  B00011100,  B00111000,  B00000000,
  B00111000,  B00011100,  B00111000,  B00000000,
  B00111000,  B00011100,  B00111000,  B00000000,
  B00111000,  B00011100,  B00111000,  B00000000,
  B00111000,  B00011100,  B00111000,  B00000000,
  B00111000,  B00011100,  B00111000,  B00000000,
  B00111000,  B00011100,  B00111000,  B00011100,
  B00111000,  B00111000,  B00011100,  B00111000,
  B00111111,  B11111000,  B00011111,  B11110000,
  B00111111,  B11110000,  B00001111,  B11110000,
  B00111111,  B11000000,  B00000011,  B11000000
};

const uint8_t PROGMEM AC_BMP[64] = {
  B00000011,  B11000000,  B00000011,  B11000000,
  B00001111,  B11110000,  B00001111,  B11110000,
  B00011111,  B11111000,  B00011111,  B11111000,
  B00011100,  B00111000,  B00011100,  B00111100,
  B00111000,  B00011100,  B00111000,  B00011100,
  B00111000,  B00011100,  B00111000,  B00000000,
  B00111000,  B00011100,  B00111000,  B00000000,
  B00111000,  B00011100,  B00111000,  B00000000,
  B00111111,  B11111100,  B00111000,  B00000000,
  B00111111,  B11111100,  B00111000,  B00000000,
  B00111111,  B11111100,  B00111000,  B00000000,
  B00111000,  B00011100,  B00111000,  B00011100,
  B00111000,  B00011100,  B00011100,  B00111000,
  B00111000,  B00011100,  B00011111,  B11111000,
  B00111000,  B00011100,  B00001111,  B11110000,
  B00111000,  B00011100,  B00000011,  B11000000
};

#endif //METER_GRAPHICS_H
