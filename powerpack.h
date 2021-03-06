/**
 * Davinci32uでパワーパックを作ってみた
 *
 * @author masuda, Masuda Naika
 */
 
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

//#define F_CPU        16000000
#define PWM_CONST    (F_CPU / 2)

//==========================================================================================
// PIN definitions, unable to use PB3(MISO) as GPIO?
//==========================================================================================
#define PIN_CS     17    // PB0, LCD CS
//#define PIN_SCK    TXLED0    // PD5, LCD SCK, MSPIM, Fixed
//#define PIN_SDA    1     // PD3, LCD SDA, MSPIM, Fixed
#define PIN_A0     8     // PB4, LCD A0
#define PIN_RST    9     // PB5, LCD RST
#define PIN_BTN    11    // PB7, Push button, pull up
#define PIN_FET1   10    // PB6, PWM OC1B, FET1, Sound, Fixed
#define PIN_FET2   13    // PC7, PWM OC4A, FET2, Motor and Light, Fixed
#define PIN_FET3   5     // PC6, FET 3, Ext power
#define PIN_CMP    7     // PE6, Comparator input, Fixed
#define PIN_ENCA   23    // PF0, Rotary encoder A, pull up
#define PIN_ENCB   22    // PF1, Rotray encoder B, pull up

//==========================================================================================
// TFT LCD Color definitions
//==========================================================================================
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
#define TRANSPARENT     -1

//==========================================================================================
// Constants
//==========================================================================================
#define TIMER1_TOP_15KHz    511
#define TIMER4_TOP_31KHZ    127
#define TIMER4_TOP_15KHZ    255

#define NOTCH_OFF   0
#define NOTCH_BE    -128

#define MODE_STOP         0
#define MODE_DRIVE        1
#define MODE_OVERLOAD     255

#define MENU_ITEM_EXIT       0
#define MENU_ITEM_LIGHT      1
#define MENU_ITEM_ACCEL      2
#define MENU_ITEM_TRAIN      3
#define MENU_ITEM_LUT        4
#define MENU_ITEM_SDUTY      5

#define DEFAULT_TRAIN_TYPE    0
#define DEFAULT_LUT_TYPE      0
#define DEFAULT_SOUND_DUTY    2
#define DEFAULT_ACCEL_VALUE   160
#define DEFAULT_LIGHT_VALUE   8

//==========================================================================================
// param values stored in EEPROM
//==========================================================================================
EEMEM uint8_t eTrainType;
EEMEM uint8_t eAccelValue;
EEMEM uint8_t eLightValue;
EEMEM uint8_t eLutType;
EEMEM uint8_t eSoundDuty;

//==========================================================================================
// メニュー項目
//==========================================================================================
PROGMEM char* menuItemPtr[] = {
    "Exit",
    "Light:",
    "Accel:",
    "Train:",
    "Lut  :",
    "SDuty:"
};

//==========================================================================================
// 速度デューティーLUT 思い通りには動かんもんだ orz
//==========================================================================================
PROGMEM uint8_t dutyLut[][256] = {
    // y=8+247/sqrt(255-1)*sqrt(x-1)
    {
      0,   8,  23,  29,  34,  38,  42,  45,  49,  51,  54,  57,  59,  61,  63,  65,
     68,  69,  71,  73,  75,  77,  79,  80,  82,  83,  85,  87,  88,  90,  91,  92,
     94,  95,  97,  98,  99, 100, 102, 103, 104, 106, 107, 108, 109, 110, 111, 113,
    114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 125, 126, 127, 128, 129, 130,
    131, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 143, 144,
    145, 146, 147, 148, 149, 150, 150, 151, 152, 153, 154, 155, 155, 156, 157, 158,
    159, 159, 160, 161, 162, 162, 163, 164, 165, 166, 166, 167, 168, 169, 169, 170,
    171, 172, 172, 173, 174, 174, 175, 176, 177, 177, 178, 179, 179, 180, 181, 181,
    182, 183, 184, 184, 185, 186, 186, 187, 188, 188, 189, 190, 190, 191, 192, 192,
    193, 193, 194, 195, 195, 196, 197, 197, 198, 199, 199, 200, 200, 201, 202, 202,
    203, 204, 204, 205, 205, 206, 207, 207, 208, 208, 209, 210, 210, 211, 211, 212,
    213, 213, 214, 214, 215, 215, 216, 217, 217, 218, 218, 219, 219, 220, 221, 221,
    222, 222, 223, 223, 224, 224, 225, 226, 226, 227, 227, 228, 228, 229, 229, 230,
    230, 231, 232, 232, 233, 233, 234, 234, 235, 235, 236, 236, 237, 237, 238, 238,
    239, 239, 240, 240, 241, 242, 242, 243, 243, 244, 244, 245, 245, 246, 246, 247,
    247, 248, 248, 249, 249, 250, 250, 251, 251, 252, 252, 253, 253, 254, 254, 255
    },

};
