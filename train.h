/**
 * Davinci32uでパワーパックを作ってみた
 *
 * ArduinoでPWMパワーパックを作ってみた しろくま氏
 * "http://www.diotown.com/creative/2011/05/arduino-de-pwm-controller.html"
 * よりデータ引用させていただきました
 * 
 * あまり鉄分濃くないのでパラメーターはアバウトです
 *
 * @author masuda, Masuda Naika
 */
 
#include <avr/pgmspace.h>

//==========================================================================================
// 列車データ構造体
//==========================================================================================
typedef struct {
    uint16_t startFreq;    // 開始周波数
    uint16_t endFreq;      // 終了周波数
    uint8_t  topSpeed;     // 切替時スピード 0-255
} PWM_DATA;

typedef struct {
    char *trainNamePtr;            // 列車名
    
    uint8_t startingAccel;         // 起動加速度　4倍値
    uint8_t constantAccelSpeed;    // 定加速度領域
    
    uint8_t *notchMaxSpeedAPtr;    // 加速ノッチ位置での最高速度配列のポインタ0-255
    uint8_t numNotchMaxSpeedDataA; // 加速ノッチの数
    uint8_t *notchAccelDataAPtr;   // 加速ノッチ係数配列のポインタ 4倍値
    uint8_t numNotchAccelDataA;    // 加速ノッチ係数配列の数
    
    uint8_t *notchAccelDataBPtr;   // 減速ノッチ係数配列のポインタ
    uint8_t numNotchAccelDataB;    // 減速ノッチ係数配列の数
    
    PWM_DATA *pwmDataAPtr;         // 加速時PWMデータ配列のポインタ
    uint8_t numPwmDataA;           // 加速時PWMデータの数
    PWM_DATA *pwmDataBPtr;         // 減速時PWMデータ配列のポインタ
    uint8_t numPwmDataB;           // 減速時PWMデータの数
} TRAIN_DATA;


//==========================================================================================
// パターン0 223系2000番台
//==========================================================================================
PROGMEM uint8_t notchMaxSpeedA0[] = {
    20, 40, 60, 80, 130
};
PROGMEM uint8_t notchAccelDataA0[] = {
    // 4倍値 {2.5*4, 3*4, 3.5*4, 4*4, 5*4}
    10, 12, 14, 16, 20
};
PROGMEM uint8_t notchAccelDataB0[] = {
    // 4倍値
    4,  8, 12, 16, 20
};
PROGMEM PWM_DATA pwmDataA0[] = {
    // 開始周波数　終了周波数　切替スピード (加速)
    {1800,    1800,      7},
    { 500,    1800,     18},
    { 500,     750,     30},
    { 600,     800,     40},
    { 550,     600,     50},
    { 250,     760,    130},
    { 760,    2550,    255}
};
PROGMEM PWM_DATA pwmDataB0[] = {
    // 開始周波数　終了周波数　切替スピード (減速)
    // 加減速時のPWM音を共用するならばデータを入れなくてもOK
};
PROGMEM TRAIN_DATA trainData0 = {
    "JRW Series 223 2000",                         // 列車名

    20,                                            // 起動加速度　4倍値
    40,                                            // 定加速度領域

    notchMaxSpeedA0,                               // 加速ノッチ位置での最高速度配列のポインタ
    sizeof(notchAccelDataA0) / sizeof(uint8_t),    // 加速ノッチの数
    notchAccelDataA0,                              // 加速ノッチ係数配列のポインタ
    sizeof(notchAccelDataA0) / sizeof(uint8_t),    // 加速ノッチ係数配列の数
    
    notchAccelDataB0,                              // 減速ノッチ係数配列のポインタ
    sizeof(notchAccelDataB0) / sizeof(uint8_t),    // 減速ノッチ係数配列の数

    pwmDataA0,                              // 加速時PWMデータ配列のポインタ
    sizeof(pwmDataA0) / sizeof(PWM_DATA),   // 加速時PWMデータの数
    pwmDataB0,                              // 減速時PWMデータ配列のポインタ
    sizeof(pwmDataB0) / sizeof(PWM_DATA)    // 減速時PWMデータの数
};

//==========================================================================================
// パターン1 京急1000
//==========================================================================================
PROGMEM uint8_t notchMaxSpeedA1[] = {
    20, 40, 60, 80, 130
};
PROGMEM uint8_t notchAccelDataA1[] = {
    10, 12, 14, 16, 20
};
PROGMEM uint8_t notchAccelDataB1[] = {
    4,  8, 12, 16, 20
};
PROGMEM PWM_DATA pwmDataA1[] = {
    { 300,     300,      10},
    { 350,     350,      11},
    { 392,     392,      12},
    { 440,     440,      13},
    { 466,     466,      14},
    { 523,     523,      15},
    { 587,     587,      16},
    { 622,     622,      17},
    { 698,     698,      18},
    { 783,     783,      19},
    { 900,     900,      35},
    { 900,    1000,      37},
    { 900,    1000,      40},
    { 300,     650,     130},
    { 650,    2550,     255}
};
PROGMEM PWM_DATA pwmDataB1[] = {
    { 900,     900,      20},
    { 900,    1000,      37},
    { 900,    1000,      40},
    { 300,     650,     130},
    { 650,    2550,     255}
};
PROGMEM TRAIN_DATA trainData1 = {
    "Keikyu 1000",
    20, 40,
    notchMaxSpeedA1, sizeof(notchMaxSpeedA1) / sizeof(uint8_t),
    notchAccelDataA1, sizeof(notchAccelDataA1) / sizeof(uint8_t),
    notchAccelDataB1, sizeof(notchAccelDataB1) / sizeof(uint8_t),
    pwmDataA1, sizeof(pwmDataA1) / sizeof(PWM_DATA),
    pwmDataB1, sizeof(pwmDataB1) / sizeof(PWM_DATA),
};

//==========================================================================================
// パターン2 207系　223系0番台　281系はるか
//==========================================================================================
PROGMEM uint8_t notchMaxSpeedA2[] = {
    20, 40, 60, 80, 130
};
PROGMEM uint8_t notchAccelDataA2[] = {
    10, 12, 14, 16, 20
};
PROGMEM uint8_t notchAccelDataB2[] = {
    4,  8, 12, 16, 20
};
PROGMEM PWM_DATA pwmDataA2[] = {
    { 800,     800,      10},
    { 380,     900,      30},
    { 500,     750,      40},
    { 500,     800,      80},
    { 800,     800,     255}
};
PROGMEM PWM_DATA pwmDataB2[] = {
};
PROGMEM TRAIN_DATA trainData2 = {
    "JRW Series 207, 223",
    20, 40,
    notchMaxSpeedA2, sizeof(notchMaxSpeedA2) / sizeof(uint8_t),
    notchAccelDataA2, sizeof(notchAccelDataA2) / sizeof(uint8_t),
    notchAccelDataB2, sizeof(notchAccelDataB2) / sizeof(uint8_t),
    pwmDataA2, sizeof(pwmDataA2) / sizeof(PWM_DATA),
    pwmDataB2, sizeof(pwmDataB2) / sizeof(PWM_DATA),
};

//==========================================================================================
// パターン3 DL
//==========================================================================================
PROGMEM uint8_t notchMaxSpeedA3[] = {
    5, 20, 40, 60, 120
};
PROGMEM uint8_t notchAccelDataA3[] = {
    10, 12, 14, 16, 20
};
PROGMEM uint8_t notchAccelDataB3[] = {
    4,  8, 12, 16, 20
};
PROGMEM PWM_DATA pwmDataA3[] = {
    { 130,     140,       5},
    { 140,     200,      50},
    { 150,     190,     100},
    { 250,     700,     120},
    { 700,    2550,     255}
};
PROGMEM PWM_DATA pwmDataB3[] = {
};
PROGMEM TRAIN_DATA trainData3 = {
    "DL",
    20, 40,
    notchMaxSpeedA3, sizeof(notchMaxSpeedA3) / sizeof(uint8_t),
    notchAccelDataA3, sizeof(notchAccelDataA3) / sizeof(uint8_t),
    notchAccelDataB3, sizeof(notchAccelDataB3) / sizeof(uint8_t),
    pwmDataA3, sizeof(pwmDataA3) / sizeof(PWM_DATA),
    pwmDataB3, sizeof(pwmDataB3) / sizeof(PWM_DATA),
};

//==========================================================================================
// パターン4 221系
//==========================================================================================
PROGMEM uint8_t notchMaxSpeedA4[] = {
    20, 40, 60, 80, 130
};
PROGMEM uint8_t notchAccelDataA4[] = {
    10, 12, 14, 16, 20
};
PROGMEM uint8_t notchAccelDataB4[] = {
    4,  8, 12, 16, 20
};
PROGMEM PWM_DATA pwmDataA4[] = {
    { 150,    1300,     130},
    {1300,    2550,     255}
};
PROGMEM PWM_DATA pwmDataB4[] = {
};
PROGMEM TRAIN_DATA trainData4 = {
    "JRW Series 211",
    20, 40,
    notchMaxSpeedA4, sizeof(notchMaxSpeedA4) / sizeof(uint8_t),
    notchAccelDataA4, sizeof(notchAccelDataA4) / sizeof(uint8_t),
    notchAccelDataB4, sizeof(notchAccelDataB4) / sizeof(uint8_t),
    pwmDataA4, sizeof(pwmDataA4) / sizeof(PWM_DATA),
    pwmDataB4, sizeof(pwmDataB4) / sizeof(PWM_DATA),
};

//==========================================================================================
//パターン5 EF210
//==========================================================================================
PROGMEM uint8_t notchMaxSpeedA5[] = {
    20, 40, 60, 80, 130
};
PROGMEM uint8_t notchAccelDataA5[] = {
    10, 12, 14, 16, 20
};
PROGMEM uint8_t notchAccelDataB5[] = {
    4,  8, 12, 16, 20
};
PROGMEM PWM_DATA pwmDataA5[] = {
    { 600,     600,      10},
    { 600,     600,      20},
    { 400,     800,      40},
    { 800,     800,     130},
    { 800,     800,     255}
};
PROGMEM PWM_DATA pwmDataB5[] = {
};
PROGMEM TRAIN_DATA trainData5 = {
    "JRF EF210",
    20, 40,
    notchMaxSpeedA5, sizeof(notchMaxSpeedA5) / sizeof(uint8_t),
    notchAccelDataA5, sizeof(notchAccelDataA5) / sizeof(uint8_t),
    notchAccelDataB5, sizeof(notchAccelDataB5) / sizeof(uint8_t),
    pwmDataA5, sizeof(pwmDataA5) / sizeof(PWM_DATA),
    pwmDataB5, sizeof(pwmDataB5) / sizeof(PWM_DATA),
};

//==========================================================================================
// パターン6 223系2000番台（新幹線用）
//==========================================================================================
PROGMEM uint8_t notchMaxSpeedA6[] = {
    40, 80, 120, 200, 255
};
PROGMEM TRAIN_DATA trainData6 = {
    "JRW Series 223 2000 (255)",
    20, 90,    // 255まで出す場合は85以上に
    notchMaxSpeedA6, sizeof(notchMaxSpeedA6) / sizeof(uint8_t),
    notchAccelDataA0, sizeof(notchAccelDataA0) / sizeof(uint8_t),
    notchAccelDataB0, sizeof(notchAccelDataB0) / sizeof(uint8_t),
    pwmDataA0, sizeof(pwmDataA0) / sizeof(PWM_DATA),
    pwmDataB0, sizeof(pwmDataB0) / sizeof(PWM_DATA),
};

//==========================================================================================
// パターン7 Tokyu 9207
// 自己満足の工作の記録 "http://jikomanzokunokosak.blog.fc2.com" を改変
//==========================================================================================
PROGMEM uint8_t notchMaxSpeedA7[] = {
    20, 40, 60, 80, 130
};
PROGMEM uint8_t notchAccelDataA7[] = {
    12, 12, 12, 16, 20
};
PROGMEM uint8_t notchAccelDataB7[] = {
    4,  8, 12, 16, 20
};
PROGMEM PWM_DATA pwmDataA7[] = {
    { 196,    196,        6},
    { 494,    622,       10},
    { 392,    740,       24},
    { 415,    740,       40},
    { 466,    587,       46},
    { 392,    523,       54},
    { 277,    587,       90},
    { 587,   1109,      130},
};
PROGMEM PWM_DATA pwmDataB7[] = {
};
PROGMEM TRAIN_DATA trainData7 = {
    "Tokyu 9207",
    20, 40,
    notchMaxSpeedA7, sizeof(notchMaxSpeedA7) / sizeof(uint8_t),
    notchAccelDataA7, sizeof(notchAccelDataA7) / sizeof(uint8_t),
    notchAccelDataB7, sizeof(notchAccelDataB7) / sizeof(uint8_t),
    pwmDataA7, sizeof(pwmDataA7) / sizeof(PWM_DATA),
    pwmDataB7, sizeof(pwmDataB7) / sizeof(PWM_DATA),
};

//==========================================================================================
// パターン8 JRE Series 231
// 自己満足の工作の記録 "http://jikomanzokunokosak.blog.fc2.com"
//==========================================================================================
PROGMEM uint8_t notchMaxSpeedA8[] = {
    20, 40, 60, 80, 130
};
PROGMEM uint8_t notchAccelDataA8[] = {
    12, 12, 12, 16, 20
};
PROGMEM uint8_t notchAccelDataB8[] = {
    4,  8, 12, 16, 20
};
PROGMEM PWM_DATA pwmDataA8[] = {
    { 2093,    2093,     12},
    { 2093,    1397,     27},
    { 1397,    2349,     29},
    { 349,     1175,    110},
    { 1175,    1245,    130}
};
PROGMEM PWM_DATA pwmDataB8[] = {
};
PROGMEM TRAIN_DATA trainData8 = {
    "JRE Series 231 Outskirts",
    20, 40,
    notchMaxSpeedA8, sizeof(notchMaxSpeedA8) / sizeof(uint8_t),
    notchAccelDataA8, sizeof(notchAccelDataA8) / sizeof(uint8_t),
    notchAccelDataB8, sizeof(notchAccelDataB8) / sizeof(uint8_t),
    pwmDataA8, sizeof(pwmDataA8) / sizeof(PWM_DATA),
    pwmDataB8, sizeof(pwmDataB8) / sizeof(PWM_DATA),
};

//==========================================================================================
// パターン9 TSURIKAKE?
//==========================================================================================
PROGMEM uint8_t notchMaxSpeedA9[] = {
    20, 40, 60, 80, 130
};
PROGMEM uint8_t notchAccelDataA9[] = {
    12, 12, 12, 16, 20
};
PROGMEM uint8_t notchAccelDataB9[] = {
    4,  8, 12, 16, 20
};
PROGMEM PWM_DATA pwmDataA9[] = {
    { 40,    1200,     255}
};
PROGMEM PWM_DATA pwmDataB9[] = {
};
PROGMEM TRAIN_DATA trainData9 = {
    "TSURIKAKE",
    20, 40,
    notchMaxSpeedA9, sizeof(notchMaxSpeedA9) / sizeof(uint8_t),
    notchAccelDataA9, sizeof(notchAccelDataA9) / sizeof(uint8_t),
    notchAccelDataB9, sizeof(notchAccelDataB9) / sizeof(uint8_t),
    pwmDataA9, sizeof(pwmDataA9) / sizeof(PWM_DATA),
    pwmDataB9, sizeof(pwmDataB9) / sizeof(PWM_DATA),
};

//==========================================================================================
// パターン10 Tokyo Metro 16000
// 東急車両製造杉村製作所 "http://blogs.yahoo.co.jp/kf7200stk"
//==========================================================================================
PROGMEM uint8_t notchMaxSpeedA10[] = {
    20, 40, 60, 80, 130
};
PROGMEM uint8_t notchAccelDataA10[] = {
    12, 12, 12, 16, 20
};
PROGMEM uint8_t notchAccelDataB10[] = {
    4,  8, 12, 16, 20
};
PROGMEM PWM_DATA pwmDataA10[] = {
    { 250,     250,      10},
    { 1000,    1000,     26},
    { 500,      900,    130}
};
PROGMEM PWM_DATA pwmDataB10[] = {
};
PROGMEM TRAIN_DATA trainData10 = {
    "Tokyo Metro 16000",
    20, 40,
    notchMaxSpeedA10, sizeof(notchMaxSpeedA10) / sizeof(uint8_t),
    notchAccelDataA10, sizeof(notchAccelDataA10) / sizeof(uint8_t),
    notchAccelDataB10, sizeof(notchAccelDataB10) / sizeof(uint8_t),
    pwmDataA10, sizeof(pwmDataA10) / sizeof(PWM_DATA),
    pwmDataB10, sizeof(pwmDataB10) / sizeof(PWM_DATA),
};

//==========================================================================================
// 列車データポインタの配列
//==========================================================================================
PROGMEM TRAIN_DATA *trains[] = {
    &trainData0,
    &trainData1,
    &trainData2,
    &trainData3,
    &trainData4,
    &trainData5,
    &trainData6,
    &trainData7,
    &trainData8,
    &trainData9,
    &trainData10,
};


