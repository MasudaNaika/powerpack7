/**
 * Davinci32u (w/ Leonardo bootloader)でパワーパックを作ってみた　３台目
 * 
 * 「ArduinoでPWMパワーパックを作ってみた」 しろくま氏
 * "http://www.diotown.com/creative/2011/05/arduino-de-pwm-controller.html"
 * にインスパイヤされ、自分も作ってみたくなった。
 *
 * 同じDuty比でもPWM周波数変化により速度・トルクが変化してしまうため、
 * モーター用に周波数一定のPWMを、励磁音用に周波数可変のPWMを用意する
 * すっきりしない実装
 *
 * @author masuda, Masuda Naika
 */

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <TFT_ILI9163C.h>
#include "powerpack.h"
#include "train.h"

//==========================================================================================
// Constants
#define UI_LOOP_DELAY        50
#define ENCODER_SHIFT        2    // １デテント４カウントなので４で割る

#define FONT_WIDTH           6
#define FONT_HEIGHT          8
#define FONT_SIZE_INFO       1
#define FONT_SIZE_SETTING    2
#define FONT_SIZE_ALERT      2
#define FONT_SIZE_NOTCH      10

#define MENU_ITEM_LENGTH     6
#define MENU_TRAIN_NAME_ROW  3

#define X_LEFT               0
#define X_CENTER             64
#define X_NOTCH_OFFSET1      34    // N
#define X_NOTCH_OFFSET2      8     // B5-B1, P1-P5

//==========================================================================================
// port and mask
//==========================================================================================
volatile uint8_t *btnPort;
volatile uint8_t btnPinMask;
volatile uint8_t *fet3Port;
volatile uint8_t fet3PinMask;
volatile uint8_t *encPortA;
volatile uint8_t encPinMaskA;
volatile uint8_t *encPortB;
volatile uint8_t encPinMaskB;

//==========================================================================================
// TFT_ILI9163
//==========================================================================================
TFT_ILI9163C lcd = TFT_ILI9163C(PIN_CS, PIN_A0, PIN_RST);

//==========================================================================================
// POWERPACK状態
//==========================================================================================
volatile struct {
    
    uint8_t trainType;        // 列車種別 0 - 10
    uint8_t accelValue;       // 0-255 加速係数　ユーザー設定値
    uint8_t accelValue2;      // 内部加速係数　(accelValue ^ 2) / 256
    uint8_t lightValue;       // 0-31/255
    uint8_t lutType ;         // LUT type
    uint8_t soundDuty;        // 0:0, 1:1/16, 2:3/32, 3:1/8
    uint8_t opMode;           // 動作モード
    int8_t notchPos;          // マスコン位置
    uint16_t trainSpeed;      // realSpeed * 256
    uint16_t frequency;       // sound frequency
    
    // LCD表示更新用変数
    int8_t dispNotchPos;
    uint16_t dispTrainSpeed;
    uint16_t dispFrequency;
    boolean forceUpdate;
    
    // 列車各種一時データ
    TRAIN_DATA *trainDataPtr;
    int8_t maxNotch;
    int8_t minNotch;

    uint8_t startingAccel;
    uint16_t constantAccelSpeed;    // realSpeed * 256
    
    uint8_t numPwmDataA;
    PWM_DATA *pwmDataAPtr;
    uint8_t numPwmDataB;
    PWM_DATA *pwmDataBPtr;
    uint8_t numNotchAccelDataB;

} PP;    // POWERPACK

//==========================================================================================
// ロータリー・エンコーダの使い方 "http://elm-chan.org/docs/tec/te04.html"
//==========================================================================================
volatile struct {
    int16_t pos;     // 軸位置
} Encoder;

inline void scanEncoder() {
    
    // 回転方向テーブル
    static const int8_t dir[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
    // インデックス
    static int8_t index;
    
    // 前回値と今回値でインデックスとする
//    index = ((index << 2) + (PINF & (_BV(PINF1) + _BV(PINF0)))) && 0x0f;
    index = (index << 2) & 0x0f;
    if (*encPortA & encPinMaskA) {
        index |= _BV(0);
    }
    if (*encPortB & encPinMaskB) {
        index |= _BV(1);
    }
    // 変化パターンから動きを得る
    int8_t n = dir[index];
    // 動きがあったら位置更新
    if (n) {
        Encoder.pos += n;
    }
}

void initEncoderPos() {
//    Encoder.pos = 0;
    Encoder.pos = 2;    // ゼロ跨ぎをうまく扱うためオフセットする
}

//==========================================================================================
// セットアップとメインループ
//==========================================================================================
void setup() {

    // ポートレジスタ、マスク設定
    btnPort = portInputRegister(digitalPinToPort(PIN_BTN));
    btnPinMask = digitalPinToBitMask(PIN_BTN);
    fet3Port = portOutputRegister(digitalPinToPort(PIN_FET3));
    fet3PinMask = digitalPinToBitMask(PIN_FET3);
    encPortA = portInputRegister(digitalPinToPort(PIN_ENCA));
    encPinMaskA = digitalPinToBitMask(PIN_ENCA);
    encPortB = portInputRegister(digitalPinToPort(PIN_ENCB));
    encPinMaskB = digitalPinToBitMask(PIN_ENCB);
    
    // コンパレーター
    ADCSRB &= ~_BV(ACME);              // 内部基準電圧
    DIDR1 |= _BV(AIN0D);               // AIN0デジタル入力禁止
    ACSR = _BV(ACIS1) + _BV(ACIS0);    // 上昇端
    ACSR |= _BV(ACIE);                 // 割り込み許可
    
    // Timerなどの設定
    motorPowerOff();
    // TIMER1 音と常点灯照明。分周なし, 位相周波数基準PWM, TOP = OC1A, 出力 = OC1B (PB6)
    TCCR1A = _BV(COM1B1) + _BV(WGM10);
    TCCR1B = _BV(WGM13) + _BV(CS10);

    // 操作読み込み、PWM出力計算などのタイマー
    // TIMER4  位相周波数基準PWM, 2分周, TOP = OCR4C, OVF割り込み許可, OC4A(PC7)出力, 15.6kHz
    TCCR4A = _BV(COM4A1) + _BV(PWM4A); // OC4A出力
    TCCR4B = _BV(CS41);                // 2分周
    TCCR4D = _BV(WGM40);               // 位相周波数基準PWM
    TCCR4E = 0;
    OCR4C = 255;                       // TOP
    TIMSK4 = _BV(TOIE4);               // OVF割り込み
    
    // PWM out pins
    pinMode(PIN_FET1, OUTPUT);
    pinMode(PIN_FET2, OUTPUT);
    
    // 12V out FET control
    pinMode(PIN_FET3, OUTPUT);
    digitalWrite(PIN_FET3, LOW);
    
    // 設定ボタン input pull-up
    pinMode(PIN_BTN, INPUT_PULLUP);
    
    // Rotary encoder input pull-up
    pinMode(PIN_ENCA, INPUT_PULLUP);
    pinMode(PIN_ENCB, INPUT_PULLUP);
    
    // SPI LCD ライブラリ内で設定される
//    pinMode(PIN_CS, OUTPUT);
//    pinMode(PIN_SCK, OUTPUT);
//    pinMode(PIN_SDA, OUTPUT);
//    pinMode(PIN_A0, OUTPUT);
//    pinMode(PIN_RST, OUTPUT);

    // LCD初期化
    lcd.begin();
    lcd.setBitrate(8000000);    // 8MHz
    lcd.setRotation(2);    // 1

    // 省電力設定
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    
    // 初期値設定
    loadParams();               // EEPROMから読み出す
    PP.trainSpeed = 0;          // realSpeed * 256
    PP.notchPos = NOTCH_OFF;    // マスコン位置
    PP.opMode = MODE_STOP;      // 動作モード

    // 割り込みを有効にする
    sei();

    // タイトル表示
    showTitle();
}

// メインループ
void loop() {
    
    volatile static uint8_t lcdCntr;    // LCD表示用カウンタ
    
    switch (PP.opMode) {
        case MODE_OVERLOAD:
            // 過電流
            alertOverload();
            // STOPモードにする
            PP.opMode = MODE_STOP;
            break;
        case MODE_DRIVE:
            if (PP.notchPos == NOTCH_BE && isBtnPressed()) {
                // 設定モードに入る
                PP.opMode = MODE_STOP;
                enterSettingMode();
            } else if (lcdCntr == 0){
                // 運転モードLCD更新 最大150msec位かかる
                updateLCD();
            }
            ++lcdCntr;
            break;
        case MODE_STOP:
            // MODE_STOP -> MODE_DRIVEにする
            if (!isOverload()) {
                enableExtPower();
                startDriveMode();
            } else {
                PP.opMode = MODE_OVERLOAD;
            }
            break;
    }
    
    // スリープさせておくが、ほとんど寝ていない
    sleep_cpu();
}

//==========================================================================================
// EEPROM save / looad
//==========================================================================================
void loadParams() {
    
    PP.trainType = DEFAULT_TRAIN_TYPE;
    PP.accelValue = DEFAULT_ACCEL_VALUE;
    PP.lightValue = DEFAULT_LIGHT_VALUE;
    PP.lutType = DEFAULT_LUT_TYPE;
    PP.soundDuty = DEFAULT_SOUND_DUTY;
    
    eeprom_busy_wait();
    uint8_t val = eeprom_read_byte(&eTrainType);
    if (val < sizeof(trains) / sizeof(TRAIN_DATA*)) {
        PP.trainType = val;
    }
    eeprom_busy_wait();
    val = eeprom_read_byte(&eAccelValue);
    if (val == 255) {
        PP.accelValue = 160;
    }
    eeprom_busy_wait();
    val = eeprom_read_byte(&eLightValue);
    if (val < 32) {
        PP.lightValue = val;
    }
    eeprom_busy_wait();
    val = eeprom_read_byte(&eLutType);
    if (val < sizeof(dutyLut) / sizeof(dutyLut[0])) {
        PP.lutType = val;
    }
    eeprom_busy_wait();
    val = eeprom_read_byte(&eSoundDuty);
    if (0 <= val & val <= 3) {
        PP.soundDuty = val;
    }
}

void saveParams() {
    
//    eeprom_busy_wait();
//    eeprom_update_byte(&eTrainType, PP.trainType); // 使えない？

    eepromUpdateByte(&eTrainType, PP.trainType);
    eepromUpdateByte(&eAccelValue, PP.accelValue);
    eepromUpdateByte(&eLightValue, PP.lightValue);
    eepromUpdateByte(&eLutType, PP.lutType);
    eepromUpdateByte(&eSoundDuty, PP.soundDuty);
}

void eepromUpdateByte(uint8_t *addr, uint8_t newValue) {
    eeprom_busy_wait();
    uint8_t val = eeprom_read_byte(addr);
    if (val != newValue) {
        eeprom_busy_wait();
        eeprom_write_byte(addr, newValue);
    }
}

//==========================================================================================
// PROGMEM内容取得関数
//==========================================================================================
// 列車データポインタ
inline TRAIN_DATA* getTrainDataPtr(uint8_t trainType) {
    return (TRAIN_DATA*) pgm_read_word(&trains[trainType]);
}
// 起動加速度 4倍値
inline uint8_t getStartingAccel() {
    return pgm_read_byte(&PP.trainDataPtr->startingAccel);
}
// 定加速度領域切り替え速度
inline uint8_t getConstantAccelSpeed() {
    return pgm_read_byte(&PP.trainDataPtr->constantAccelSpeed);
}

// 加速ノッチ最大速度の数
inline uint8_t getNumNotchMaxSpeedA() {
    return pgm_read_byte(&PP.trainDataPtr->numNotchMaxSpeedDataA);
}
// 加速ノッチ最大速度の値
inline uint8_t getNotchMaxSpeedA(int8_t notchPos) {
    uint8_t *notchMaxSpeedAPtr = (uint8_t*) pgm_read_word(&PP.trainDataPtr->notchMaxSpeedAPtr);
    return pgm_read_byte(&notchMaxSpeedAPtr[notchPos - 1]);
}
// 加速ノッチ加速係数の数
inline uint8_t getNumNotchAccelDataA() {
    return pgm_read_byte(&PP.trainDataPtr->numNotchAccelDataA);
}
// 加速ノッチ加速係数の値
inline uint8_t getNotchAccelDataA(int8_t notchPos) {
    uint8_t *notchAccelDataAPtr = (uint8_t*) pgm_read_word(&PP.trainDataPtr->notchAccelDataAPtr);
    return pgm_read_byte(&notchAccelDataAPtr[notchPos - 1]);
}

// 減速ノッチ加速係数の数
inline uint8_t getNumNotchAccelDataB() {
    return pgm_read_byte(&PP.trainDataPtr->numNotchAccelDataB);
}
// 減速ノッチ減速係数の値
inline uint8_t getNotchAccelDataB(int8_t notchPos) {
    uint8_t *notchAccelDataBPtr = (uint8_t*) pgm_read_word(&PP.trainDataPtr->notchAccelDataBPtr);
    return pgm_read_byte(&notchAccelDataBPtr[-notchPos - 1]);
}

// 加速PWMデータの数
inline uint8_t getNumPwmDataA() {
    return pgm_read_byte(&PP.trainDataPtr->numPwmDataA);
}
// 減速PWMデータの数
inline uint8_t getNumPwmDataB() {
    return pgm_read_byte(&PP.trainDataPtr->numPwmDataB);
}
// 加速PWMデータポインタ
inline PWM_DATA* getPwmDataAPtr() {
    return (PWM_DATA*) pgm_read_word(&PP.trainDataPtr->pwmDataAPtr);
}
// 減速PWMデータポインタ
inline PWM_DATA* getPwmDataBPtr() {
    return (PWM_DATA*) pgm_read_word(&PP.trainDataPtr->pwmDataBPtr);
}
// PWMデータ、切替時スピード
inline uint8_t getPwmDataTopSpeed(PWM_DATA *pwmDataPtr) {
    return pgm_read_byte(&pwmDataPtr->topSpeed);
}
// PWMデータ、開始周波数
inline uint16_t getPwmDataStartFreq(PWM_DATA *pwmDataPtr) {
    return pgm_read_word(&pwmDataPtr->startFreq);
}
// PWMデータ、終了周波数
inline uint16_t getPwmDataEndFreq(PWM_DATA *pwmDataPtr) {
    return pgm_read_word(&pwmDataPtr->endFreq);
}

// LUT値
inline uint8_t getLutValue(uint8_t lutType, uint8_t duty) {
    return pgm_read_byte(&dutyLut[lutType - 1][duty]);
}

// 設定メニュー項目ポインタ
inline char* getMenuItemPtr(uint8_t index) {
    return (char*) pgm_read_word(&menuItemPtr[index]);
}
// 列車名ポインタ
inline char* getTrainNamePtr(TRAIN_DATA *trainDataPtr) {
    return (char*) pgm_read_word(&trainDataPtr->trainNamePtr);
}

//==========================================================================================
// 入出力関連
//==========================================================================================
// 過電流チェック
inline boolean isOverload() {
    return (ACSR & _BV(ACO)) != 0;
}

// 設定ボタン押下チェック
inline boolean isBtnPressed() {
//    return (PINB & _BV(PINB7)) == 0;
//    return digitalRead(PIN_BTN) == 0;
    return (*btnPort & btnPinMask) == 0;
}

// 外部出力許可
inline void enableExtPower() {
//    PORTC |= _BV(PINC6);
//    digitalWrite(PIN_FET3, HIGH);
    *fet3Port |= fet3PinMask;
}

// 出力停止
inline void shutdownPower() {
    OCR4A = 0;
    OCR1B = 0;
//    PORTC &= ~_BV(PINC6);
//    digitalWrite(PIN_FET3, LOW);
    *fet3Port &= ~fet3PinMask;
}

// モーター・照明出力停止
inline void motorPowerOff() {
    OCR4A = 0;
    OCR1B = 0;
}

// 走行用PWM出力, PWM4
inline void setMotorPwm() {
    if (PP.lutType == 0) {
        OCR4A = PP.trainSpeed >> 8;
    } else {
        // LUTを参照して低速時電圧を補正する
        OCR4A = getLutValue(PP.lutType, PP.trainSpeed >> 8);
    }
}

// 常点灯照明, PWM1
inline void setLightPwm(uint8_t value) {
    setPrescaler1();
    OCR1B = value;            // 0/256 - 15/256
    OCR1A = 255;              // 31kHz
}

// TIMER1、プリスケーラー、前置分周なし
inline void setPrescaler1() {
    if ((TCCR1B & (_BV(CS12) + _BV(CS11) + _BV(CS10))) != _BV(CS10)) {
        TCCR1B &= ~(_BV(CS12) + _BV(CS11) + _BV(CS10));
        TCCR1B |= _BV(CS10);
    }
}

// TIMER1、プリスケーラー、８分周　低周波数用
inline void setPrescaler8() {
    if ((TCCR1B & (_BV(CS12) + _BV(CS11) + _BV(CS10))) != _BV(CS11)) {
        TCCR1B &= ~(_BV(CS12) + _BV(CS11) + _BV(CS10));
        TCCR1B |= _BV(CS11);
    }
}

// 指定周波数のPWM音出力, PWM1
inline void setSoundPwm() {
    
    if (PP.frequency == 0) {
        setCoastingPwm();
        return;
    }
    
    uint16_t top;
    if (PP.frequency < 128) {
        // 周波数が低ければ分周比を変える, ８分周
        setPrescaler8();
        top = (PWM_CONST >> 3) / PP.frequency;
    } else {
        // 前置分周なし
        setPrescaler1();
        top = PWM_CONST / PP.frequency;
    }

    // 音のPWM duty 0, 1/16, 3/32, 1/8
    switch(PP.soundDuty) {
        case 1:
            OCR1B = top >> 4;
            break;
        case 2:
            OCR1B = (top >> 4) + (top >> 5);
            break;
        case 3:
            OCR1B = top >> 3;
            break;
        default:
            OCR1B = 0;
            break;
    }
    OCR1A = top;
}

// 音階を鳴らさない場合もPWM1から出力する。音の有無で速度差を生じないように
inline void setCoastingPwm() {
    setPrescaler1();
    switch(PP.soundDuty) {
        case 1:
            OCR1B = 31;
            break;
        case 2:
            OCR1B = 47;
            break;
        case 3:
            OCR1B = 63;
            break;
        default:
            OCR1B = 0;
            break;
    }
    OCR1A = 511;            // 15.6kHz
}

// テーブルを参照して速度に応じた励磁音出力を行う
void setSound() {
    
    PWM_DATA *pwmDataPtr;
    uint8_t dataLength;
    
    // 減速の場合データがあればそれを使う
    if (PP.notchPos < 0 && PP.numPwmDataB > 0) {
        dataLength = PP.numPwmDataB;
        pwmDataPtr = PP.pwmDataBPtr;
    } else {
        dataLength = PP.numPwmDataA;
        pwmDataPtr = PP.pwmDataAPtr;
    }
    
    uint16_t bottomSpeed = 0;
    boolean found = false;
    for (uint8_t i = 0; i < dataLength; ++i) {
        uint16_t topSpeed = getPwmDataTopSpeed(pwmDataPtr) << 8;
        if (PP.trainSpeed <= topSpeed) {
            uint16_t startFreq = getPwmDataStartFreq(pwmDataPtr);
            uint16_t endFreq = getPwmDataEndFreq(pwmDataPtr);
            int16_t gap = endFreq - startFreq;
            if (topSpeed > bottomSpeed) {
                PP.frequency = startFreq + (int32_t) 
                    gap * (PP.trainSpeed - bottomSpeed) / (topSpeed - bottomSpeed);
                setSoundPwm();
                found = true;
                break;
            }
        }
        bottomSpeed = topSpeed;
        ++pwmDataPtr;
    }

    if (!found) {
        PP.frequency = 0;
        setCoastingPwm();
    }
}

//==========================================================================================
// UI
//==========================================================================================
// タイトル表示
void showTitle() {

    // LCD表示
    lcd.clearScreen();
    lcd.setTextColor(YELLOW, BLACK);
    lcd.setCursor(0, 16);
    lcd.setTextSize(FONT_SIZE_SETTING);
    lcd.println("DAVINCI32U");
    lcd.println("    de\n");
    lcd.print  ("POWERPACK!");
    delay(2000);
}

// マスコン位置の中立をセットする
boolean initEncoderPosition() {

    // LCD表示
    lcd.clearScreen();
    lcd.setTextColor(CYAN, BLACK);
    lcd.setCursor(0, 16);
    lcd.setTextSize(FONT_SIZE_SETTING);
    lcd.println("Set Mascon");
    lcd.println("to NEUTRAL");
    lcd.print("position.");
    
    while (!isBtnPressed()) {
        if (!delay2(UI_LOOP_DELAY)) {
            return false;
        }
    }
    waitButtonRelease();
    
    initEncoderPos();
    PP.notchPos = NOTCH_OFF;
    return true;
}

// ボタンが離されるのを待つ
void waitButtonRelease() {
    while(isBtnPressed()) {
        if (!delay2(UI_LOOP_DELAY)) {
            return;
        }
    }
}

// 過電流警告
void alertOverload() {
    
    lcd.clearScreen();
    lcd.setCursor(0, 32);
    lcd.setTextSize(FONT_SIZE_ALERT);
    lcd.setTextColor(MAGENTA, BLACK);
    lcd.println(" * ALERT *");
    lcd.println("Current");
    lcd.print("overload!!");

    uint8_t i = 0;
    boolean flg = true;
    while (!isBtnPressed() | isOverload()) {    // ここは"||"じゃない
        delay(UI_LOOP_DELAY);
        if ((i & 0x7) == 0) {
            lcd.invertDisplay(flg);
            flg = !flg;
        }
        ++i;
    }
    lcd.invertDisplay(false);
    waitButtonRelease();
    delay(500);
}

// 過電流チェックつきdelay
boolean delay2(uint16_t delayms) {
    
    uint32_t t = millis();
    uint32_t te = t + delayms ;
    boolean carry = (te < t);

    while (carry || t <= te) {
        if (PP.opMode == MODE_OVERLOAD) {
            return false;
        }
        t = millis();
        carry &= (te < t);
    }
    return true;
}

// 右揃えで数値表示
void formattedPrint(uint16_t value, uint8_t numDigits) {
    uint8_t i = 0;
    if (value < 10) {
        i = 1;
    } else if (value < 100) {
        i = 2;
    } else if (value < 1000) {
        i = 3;
    } else if (value < 10000) {
        i = 4;
    } else {
        i = 5;
    }
    for ( ; i < numDigits; ++i) {
        lcd.print(" ");
    }
    lcd.print(value);
}

//==========================================================================================
// 運転関連
//==========================================================================================
// 運転モード開始
void startDriveMode() {
    
    // エンコーダー位置を初期化する
    if (!initEncoderPosition()) {
        return;
    }
    
    // 変数設定
    PP.forceUpdate = true;
    PP.trainSpeed = 0;
    PP.accelValue2 = (PP.accelValue * PP.accelValue) >> 8;
    
    PP.trainDataPtr = getTrainDataPtr(PP.trainType);
    PP.maxNotch = getNumNotchAccelDataA();
    PP.numNotchAccelDataB = getNumNotchAccelDataB();
    if (PP.numNotchAccelDataB == 0) {
        PP.minNotch = -PP.maxNotch;
    } else {
        PP.minNotch = -PP.numNotchAccelDataB;
    }
    PP.numPwmDataA = getNumPwmDataA();
    PP.pwmDataAPtr = getPwmDataAPtr();
    PP.numPwmDataB = getNumPwmDataB();
    PP.pwmDataBPtr = getPwmDataBPtr();
    
    PP.startingAccel = getStartingAccel();
    PP.constantAccelSpeed = getConstantAccelSpeed() << 8;

    // LCD表示
    showBaseDriveScreen();
    
    PP.opMode = MODE_DRIVE;
}

// 運転モードLCD基本表示
void showBaseDriveScreen() {
    
    lcd.clearScreen();
    lcd.setTextSize(FONT_SIZE_INFO);
    // 列車タイプ
    lcd.setTextColor(MAGENTA, BLACK);
    lcd.setCursor(0, FONT_SIZE_NOTCH * FONT_HEIGHT);
    lcd.print("Train:");
    lcd.print(getTrainNamePtr(PP.trainDataPtr));
    // Light
    lcd.setTextColor(WHITE, BLACK);
    lcd.setCursor(X_LEFT, FONT_SIZE_NOTCH * FONT_HEIGHT 
        + FONT_SIZE_INFO * FONT_HEIGHT * MENU_TRAIN_NAME_ROW);
    lcd.print("Light:");
    formattedPrint(PP.lightValue, 3);
    // Accel
    lcd.setCursor(X_CENTER, FONT_SIZE_NOTCH * FONT_HEIGHT 
        + FONT_SIZE_INFO * FONT_HEIGHT * MENU_TRAIN_NAME_ROW);
    lcd.print("Accel:");
    formattedPrint(PP.accelValue, 3);
    // LUT
    lcd.setCursor(X_LEFT, FONT_SIZE_NOTCH * FONT_HEIGHT 
        + FONT_SIZE_INFO * FONT_HEIGHT * (MENU_TRAIN_NAME_ROW + 1));
    lcd.print("Lut  :");
    formattedPrint(PP.lutType, 3);
    // SoundDuty
    lcd.setCursor(X_CENTER, FONT_SIZE_NOTCH * FONT_HEIGHT 
        + FONT_SIZE_INFO * FONT_HEIGHT * (MENU_TRAIN_NAME_ROW + 1));
    lcd.print("SDuty:");
    formattedPrint(PP.soundDuty, 3);
    // MotorDuty
    lcd.setCursor(X_LEFT, FONT_SIZE_NOTCH * FONT_HEIGHT 
        + FONT_SIZE_INFO * FONT_HEIGHT * (MENU_TRAIN_NAME_ROW + 2));
    lcd.print("MDuty:");
    // Sound Freq
    lcd.setCursor(X_CENTER, FONT_SIZE_NOTCH * FONT_HEIGHT 
        + FONT_SIZE_INFO * FONT_HEIGHT * (MENU_TRAIN_NAME_ROW + 2));
    lcd.print("Freq:");
}

// 運転状態表示更新
void updateLCD() {
    
    // sound freq
    if (PP.forceUpdate || PP.dispFrequency != PP.frequency) {
        // 途中で割り込みによりPP変数が変化することがあるので一時変数を利用する
        uint16_t value = PP.frequency;
        lcd.setTextColor(WHITE, BLACK);
        lcd.setTextSize(FONT_SIZE_INFO);
        // "Freq:".length() = 5;
        lcd.setCursor(X_CENTER + FONT_WIDTH * 5, FONT_SIZE_NOTCH * FONT_HEIGHT 
            + FONT_SIZE_INFO * FONT_HEIGHT * (MENU_TRAIN_NAME_ROW + 2));
        formattedPrint(value, 5);
        PP.dispFrequency = value;
    }
    
    // motor duty
    if (PP.forceUpdate || PP.dispTrainSpeed != PP.trainSpeed) {
        uint16_t value = PP.trainSpeed;
        // 停止した場合はニュートラルノッチ色を緑に変えるため強制変更フラグを立てる
        if (PP.dispNotchPos == NOTCH_OFF && value == 0) {
            PP.forceUpdate = true;
        }
        lcd.setTextColor(WHITE, BLACK);
        lcd.setTextSize(FONT_SIZE_INFO);
        // "MDuty:".length() = 6
        lcd.setCursor(FONT_WIDTH * 6, FONT_SIZE_NOTCH * FONT_HEIGHT 
            + FONT_SIZE_INFO * FONT_HEIGHT * (MENU_TRAIN_NAME_ROW + 2));
        formattedPrint(value >> 8, 3);
        PP.dispTrainSpeed = value;
    }
    
    // ノッチ表示する
    if (PP.forceUpdate || PP.dispNotchPos != PP.notchPos) {
        int8_t value = PP.notchPos;    
        lcd.setTextSize(FONT_SIZE_NOTCH);
        if (value == NOTCH_BE) {
            // 非常停止
            lcd.setCursor(X_NOTCH_OFFSET2, 0);
            lcd.setTextColor(RED, BLACK);
            lcd.print("BE");
        } else if (value > NOTCH_OFF) {
            // 加速ノッチ
            lcd.setCursor(X_NOTCH_OFFSET2, 0);
            lcd.setTextColor(YELLOW, BLACK);
            lcd.print("P");
            lcd.print(value);
        } else if (value < NOTCH_OFF) {
            // 減速ノッチ
            lcd.setCursor(X_NOTCH_OFFSET2, 0);
            lcd.setTextColor(RED, BLACK);
            lcd.print("B");
            lcd.print(-value);
        } else {
            // ニュートラル
            if (PP.dispNotchPos != NOTCH_OFF) {
                lcd.fillRect(X_NOTCH_OFFSET2, 0, X_NOTCH_OFFSET1 - X_NOTCH_OFFSET2, 
                    (FONT_HEIGHT - 1) * FONT_SIZE_NOTCH, BLACK);
                lcd.fillRect(X_NOTCH_OFFSET1 + FONT_WIDTH * FONT_SIZE_NOTCH, 0,
                    X_NOTCH_OFFSET2 + FONT_WIDTH * FONT_SIZE_NOTCH + (FONT_WIDTH - 1) * FONT_SIZE_NOTCH
                        - (X_NOTCH_OFFSET1 + FONT_WIDTH * FONT_SIZE_NOTCH), 
                    (FONT_HEIGHT - 1) * FONT_SIZE_NOTCH, BLACK);
            }
            lcd.setCursor(X_NOTCH_OFFSET1, 0);
            if (PP.dispTrainSpeed == 0) {
                // 完全停止時は緑
                lcd.setTextColor(GREEN, BLACK);
            } else {
                // その他は黄色
                lcd.setTextColor(YELLOW, BLACK);
            }
            lcd.print("N");
        }
        PP.dispNotchPos = value;
    }
    // 強制変更フラグを消す
    PP.forceUpdate = false;
}

// ノッチ・速度に応じた出力制御をおこなう 約256Hz
void driveTrain() {
    
    // Encoder.pos -> masconNotch
    int16_t encoderPos = Encoder.pos >> ENCODER_SHIFT;

    if (encoderPos > PP.maxNotch) {
        PP.notchPos = PP.maxNotch;
    } else if (encoderPos < PP.minNotch) {
        PP.notchPos = NOTCH_BE;
    } else {
        PP.notchPos = encoderPos;
    }
    
    // 非常停止
    if (PP.notchPos == NOTCH_BE) {
        motorPowerOff();
        PP.trainSpeed = 0;
        PP.frequency = 0;
        return;
    }
    
    // 加減速処理
    if (PP.notchPos > NOTCH_OFF) {
        // 速度変化量を計算する
        // 加速度1km/h/s -> (1 * 4) * (128 * 128 / 256) / 256 = 1
        // 加速度計算テキトーに簡略化, a = 加速度, v = 速度, As = 初期加速度, Vc = 定加速度領域終了速度
        // a = As (0 <= v <= Vc), a = As * Vc / v (Vc < v)
        uint8_t accel = (PP.trainSpeed <= PP.constantAccelSpeed)
             ? min(PP.startingAccel, getNotchAccelDataA(PP.notchPos))
             : PP.startingAccel * (PP.constantAccelSpeed >> 8) / (PP.trainSpeed >> 8);
        uint16_t delta = (accel * PP.accelValue2) >> 8;

        // ノッチの最高速度を取得する
        uint16_t notchMaxSpeed = getNotchMaxSpeedA(PP.notchPos) << 8;
        
        // 加速
        if (notchMaxSpeed < PP.trainSpeed) {
            // do nothing
        } else if (notchMaxSpeed - PP.trainSpeed > delta) {
            PP.trainSpeed += delta;
        } else {
            PP.trainSpeed = notchMaxSpeed;
        }
    } else if (PP.notchPos < NOTCH_OFF) {
        // 速度変化量を計算する
        uint16_t delta = PP.numNotchAccelDataB > 0
            ? (getNotchAccelDataB(PP.notchPos) * PP.accelValue2) >> 8
            : (getNotchAccelDataA(PP.notchPos) * PP.accelValue2) >> 8;
        // 減速
        if (PP.trainSpeed > delta) {
            PP.trainSpeed -= delta;
        } else {
            PP.trainSpeed = 0;
        }
    } else {
        // ノッチオフ 停止直前の場合は徐々に減らす。
        // 見かけ上停止したまま電流が流れ続けるのを回避するため
        if (PP.trainSpeed > 0 && PP.trainSpeed < 512) {
            --PP.trainSpeed;
        }
    }
    
    // TIMER4, モーターPWM出力
    setMotorPwm();
    
    // TIMER1, 励磁音 or 照明
    if (PP.trainSpeed == 0) {
        // 停止時照明
        setLightPwm(PP.lightValue);
        PP.frequency = 0;
    } else if (PP.notchPos == NOTCH_OFF) {
        // 惰行
        setCoastingPwm();
        PP.frequency = 0;
    } else {
        // 励磁音
        setSound();
    }
}

//==========================================================================================
// 設定関連
//==========================================================================================
// メニュー項目を表示する
void showMenuItem(uint8_t menuItem, boolean invert) {
    
    lcd.setTextSize(FONT_SIZE_SETTING);
    if (invert) {
        lcd.setTextColor(BLACK, WHITE);
    } else {
        lcd.setTextColor(WHITE, BLACK);
    }
    uint8_t y = FONT_HEIGHT * FONT_SIZE_SETTING * menuItem;
    if (menuItem > MENU_ITEM_TRAIN) {
        y += FONT_HEIGHT * FONT_SIZE_INFO * MENU_TRAIN_NAME_ROW;
    }
    lcd.setCursor(X_LEFT, y);
    lcd.print(getMenuItemPtr(menuItem));
}

// 項目数値を表示する
void showItemValue(uint8_t menuItem, uint8_t value, boolean invert) {
    
    lcd.setTextSize(FONT_SIZE_SETTING);
    if (invert) {
        lcd.setTextColor(BLACK, WHITE);
    } else {
        lcd.setTextColor(WHITE, BLACK);
    }
    uint8_t y = FONT_HEIGHT * FONT_SIZE_SETTING * menuItem;
    if (menuItem > MENU_ITEM_TRAIN) {
        y += FONT_HEIGHT * FONT_SIZE_INFO * MENU_ITEM_TRAIN;
    }
    lcd.setCursor(X_LEFT + FONT_WIDTH * FONT_SIZE_SETTING * MENU_ITEM_LENGTH, y);
    formattedPrint(value, 3);
}

// メニューに列車名を表示する
void showTrainType(uint8_t value) {
    lcd.fillRect(0, FONT_HEIGHT * FONT_SIZE_SETTING * (MENU_ITEM_TRAIN + 1)
        , 128, FONT_HEIGHT * FONT_SIZE_INFO * MENU_TRAIN_NAME_ROW, BLACK);
    lcd.setTextSize(FONT_SIZE_INFO);
    lcd.setTextColor(WHITE, BLACK);
    lcd.setCursor(X_LEFT, FONT_HEIGHT * FONT_SIZE_SETTING * (MENU_ITEM_TRAIN + 1));
    lcd.print(getTrainNamePtr(getTrainDataPtr(value)));
}

// 設定モード
void enterSettingMode() {

    // ボタンが離されるのを待つ
    waitButtonRelease();

    // show menu items
    lcd.clearScreen();
    lcd.setTextSize(FONT_SIZE_SETTING);
    
    // EXIT
    showMenuItem(MENU_ITEM_EXIT, false);
    // Light
    showMenuItem(MENU_ITEM_LIGHT, false);
    showItemValue(MENU_ITEM_LIGHT, PP.lightValue, false);
    // Accel
    showMenuItem(MENU_ITEM_ACCEL, false);
    showItemValue(MENU_ITEM_ACCEL, PP.accelValue, false);
    // Train
    showMenuItem(MENU_ITEM_TRAIN, false);
    showItemValue(MENU_ITEM_TRAIN, PP.trainType, false);
    showTrainType(PP.trainType);
    // Lut
    showMenuItem(MENU_ITEM_LUT, false);
    showItemValue(MENU_ITEM_LUT, PP.lutType, false);
    // SDuty
    showMenuItem(MENU_ITEM_SDUTY, false);
    showItemValue(MENU_ITEM_SDUTY, PP.soundDuty, false);
    
    int16_t newValue = 0;
    int16_t lastValue = 0;
    
    do {
        // エンコーダー原点設定
        initEncoderPos();
        int16_t lastEncPos = Encoder.pos >> ENCODER_SHIFT;
        
        // 項目反転
        showMenuItem(newValue, true);
        
        while (!isBtnPressed()) {
            boolean moved = false;
            // ここのロータリーエンコーダー処理は相対位置
            int16_t encoderPos = Encoder.pos >> ENCODER_SHIFT;
            if (lastEncPos < encoderPos && newValue < 5) {
                ++newValue;
                moved = true;
            } else if (lastEncPos > encoderPos && newValue > 0) {
                --newValue;
                moved = true;
            }
            lastEncPos = encoderPos;
            
            if (moved) {
                // 反転解除
                showMenuItem(lastValue, false);
                // 項目反転
                showMenuItem(newValue, true);
            }
            lastValue = newValue;
    
            if (!delay2(UI_LOOP_DELAY)) {
                return;
            }
        }
        
        // ボタンが離されるのを待つ
        waitButtonRelease();
        
        // 反転解除
        if (newValue > 0) {
            showMenuItem(newValue, false);
        }
        
        boolean flg = true;
        switch (newValue) {
            case 1:
                // 停止時照明選択
                flg = setSelectedValue(MENU_ITEM_LIGHT, &PP.lightValue , 0, 31);
                break;
            case 2:
                // 加減速度選択
                flg = setSelectedValue(MENU_ITEM_ACCEL, &PP.accelValue, 0, 255);
                break;
            case 3:
                // 列車タイプを選択する
                flg = setSelectedValue(MENU_ITEM_TRAIN, &PP.trainType, 0, sizeof(trains) / sizeof(TRAIN_DATA*) - 1);
                break;
            case 4:
                // LUT選択
                flg = setSelectedValue(MENU_ITEM_LUT, &PP.lutType, 0, sizeof(dutyLut) / sizeof(dutyLut[0]));
                break;
            case 5:
                // 励磁音pulse width選択 0, 1/16, 3/32, 1/8
                flg = setSelectedValue(MENU_ITEM_SDUTY, &PP.soundDuty, 0, 3);
                break;
            default:
                break;
        }
        if (!flg) {
            return;
        }
    } while (newValue != MENU_ITEM_EXIT);
    
    // EEPROMに保存する
    saveParams();
}

// 各種設定値をセットする
boolean setSelectedValue(uint8_t menuItem, volatile uint8_t *value, uint8_t minValue, uint8_t maxValue) {
    
    // エンコーダー原点設定
    initEncoderPos();
    int16_t lastEncPos = Encoder.pos >> ENCODER_SHIFT;
    
    int16_t newValue = *value;
    int16_t lastValue = *value;
    
    // 数値反転
    showItemValue(menuItem, lastValue, true);
    
    while (!isBtnPressed()) {
        
        boolean moved = false;
        // ここのロータリーエンコーダー処理は相対位置
        int16_t encoderPos = Encoder.pos >> ENCODER_SHIFT;
        if (lastEncPos < encoderPos && newValue < maxValue) {
            ++newValue;
            moved = true;
        } else if (lastEncPos > encoderPos && newValue > minValue) {
            --newValue;
            moved = true;
        }
        lastEncPos = encoderPos;
        
        if (moved) {
            if (lastValue != newValue) {
                // 数値表示
                showItemValue(menuItem, newValue, true);
                lastValue = newValue;
                // メニュー項目特異処理
                // 明るさを確認するため出力させる
                if (menuItem == MENU_ITEM_LIGHT) {
                    setLightPwm((uint8_t) newValue);
                }
                // 列車種別を表示する
                if (menuItem == MENU_ITEM_TRAIN) {
                    showTrainType(newValue);
                }
            }
            lastValue = newValue;
        }
        if (!delay2(UI_LOOP_DELAY)) {
            return false;
        }
    }
    
    // ボタンが離されるのを待つ
    waitButtonRelease();
    
    // 照明off
    if (menuItem == MENU_ITEM_LIGHT) {
        setLightPwm(0);
    }
    
    // 新しい値をセットする
    *value = newValue;
    
    // 数値反転を戻す
    showItemValue(menuItem, newValue, false);
    
    return true;
}

//==========================================================================================
// 割り込み
//==========================================================================================
// コンパレーター割り込み、過電流検知
ISR(ANALOG_COMP_vect) {
    shutdownPower();
    PP.opMode = MODE_OVERLOAD;
}

// TIMER4 overflow割り込み 15.6kHz
ISR(TIMER4_OVF_vect) {
    
    volatile static uint8_t timerCntr;    // TIMER4カウンタ
    volatile static uint8_t scanCntr;     // ロータリーエンコーダー用カウンタ

    // ロータリーエンコーダースキャン 15.6kHz / 8 = 1.95kHz
    // 絶対位置をずらさないために速め 1.5usec位かかる
    if ((++scanCntr & 0x07) == 0) {
        scanEncoder();
    }
    
    // 約256Hz周期で処理する
    // 16MHz / 2分周 / 510(位相周波数基準PWM周期) / 256Hz = 61
    if (timerCntr == 0 && PP.opMode == MODE_DRIVE) {
        // 運転モードの場合は出力制御をする 120usec位かかる
         driveTrain();
    }
    if (++timerCntr == 61) {    // 58?
        timerCntr = 0;
    }
}
