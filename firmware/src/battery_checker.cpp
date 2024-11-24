#define F_CPU 1200000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

static constexpr int NUM_LEDS = 5;
static constexpr uint8_t LED_PIN_MASK = (1 << NUM_LEDS) - 1;

// 線形変換パラメータの精度
// static constexpr int FIXED_PREC = 12;
// static constexpr uint32_t FIXED_ONE = 1ul << FIXED_PREC;

// 電圧の精度
static constexpr int VOLT_PREC = 8;
static constexpr uint16_t VOLT_ONE = 1ul << VOLT_PREC;

// レベル表示の階調
static constexpr uint8_t LEVEL_STEP = 5;
static constexpr uint8_t LEVEL_RANGE = NUM_LEDS * LEVEL_STEP;
static constexpr uint8_t LEVEL_MIN = 1;
static constexpr uint8_t LEVEL_MAX = LEVEL_MIN + LEVEL_RANGE - 1;

static constexpr uint16_t ADC_VCC = 1024;

// プローブ開放状態の ADC 値範囲
static constexpr uint16_t ADC_OPEN_MIN = 650; // VOLT_VCC_MAX に対応
static constexpr uint16_t ADC_OPEN_MAX = 774; // VOLT_VCC_MIN に対応

// VCC の電圧範囲
static constexpr uint16_t VOLT_VCC_MIN = 2.5f * VOLT_ONE; // ADC_OPEN_MAX に対応
static constexpr uint16_t VOLT_VCC_MAX = 3.2f * VOLT_ONE; // ADC_OPEN_MIN に対応

// プローブ開放判定の閾値電圧
static constexpr uint16_t VOLT_OPEN_MAX = 0.1f * VOLT_ONE;

// 1.5V 電池の電圧範囲
static constexpr uint16_t VOLT_1V5_MIN = 1.0f * VOLT_ONE;
static constexpr uint16_t VOLT_1V5_MAX = 1.6f * VOLT_ONE;

// 3V 電池の電圧範囲
static constexpr uint16_t VOLT_3V0_MIN = 2.5f * VOLT_ONE;
static constexpr uint16_t VOLT_3V0_MAX = 3.1f * VOLT_ONE;

// 3.7V 電池の電圧範囲
static constexpr uint16_t VOLT_3V7_MIN = 3.6f * VOLT_ONE;
static constexpr uint16_t VOLT_3V7_MAX = 4.2f * VOLT_ONE;

// 9V 電池の電圧範囲
static constexpr uint16_t VOLT_9V0_MIN = 7.0f * VOLT_ONE;
static constexpr uint16_t VOLT_9V0_MAX = 9.0f * VOLT_ONE;

static constexpr uint16_t RH = 10;
static constexpr uint16_t RL = 1;

// // ADC 値から自己バッテリ電圧への変換係数
// static constexpr int32_t ADC2LVL_MYBAT_SLOPE = LEVEL_RANGE * FIXED_ONE * ADC2VOLT_OPEN_SLOPE_FLOAT / VOLT_3V0_RANGE;
// static constexpr int32_t ADC2LVL_MYBAT_INTERSEPT = (ADC2VOLT_OPEN_INTERSEPT_FLOAT - VOLT_3V0_MIN) * FIXED_ONE * LEVEL_RANGE / VOLT_3V0_RANGE;

// LED ピン番号
//static constexpr uint8_t ledPins[] = { 0, 1, 2, 3, 4 };
#define LED_PINS(iled) (iled)

// ADC 番号
static constexpr uint8_t ADC_IN = 0;

//
static constexpr uint8_t PIN_ADC_GND = 4;

// ステート
enum class State : uint8_t {
  WAIT_OPEN,
  OPEN,
  MEAS,
};
static State state = State::WAIT_OPEN;

// 電圧レンジ
enum class VoltRange : uint8_t {
  OPEN,
  BAT_1V5,
  BAT_3V0,
  BAT_3V7,
  BAT_9V0,
};
static VoltRange range;

// 電源電圧
static uint16_t voltVcc;

static constexpr uint8_t ADC_OPEN_SAMPLE_INTERVAL = (1 << 4);
static constexpr uint8_t ADC_OPEN_HISTORY_SIZE = 2;
static uint8_t adcOpenSampleCounter = 0;
static uint16_t adcOpenHistory[ADC_OPEN_HISTORY_SIZE];
static uint8_t adcOpenHistoryIndex = 0;
//static uint16_t adcOpen0;
//static uint16_t adcOpen1;

// 1N4148W の Vf * 3
static uint16_t volt3xVf;
static uint16_t adc3xVf;

// 各 LED のレベル値
static uint8_t ledValues[NUM_LEDS];

static constexpr int LED_DRIVE_INTERVAL_MS = 2;
static constexpr int LED_PHASE_STEP_PREC = 6;
static constexpr uint16_t LED_PHASE_PERIOD = NUM_LEDS * (1ul << LED_PHASE_STEP_PREC);
static uint16_t ledPhase = 0;

static uint16_t tickCount = 0;

static void loop(void);
static uint16_t readAdc();
static uint16_t analogRead();
static void estimateVcc(uint16_t adcVal);
static void meas(uint16_t adcVal);
static uint16_t adc2Volt(uint16_t adcVal);
static void setVoltageToLeds(VoltRange range, uint16_t volt);
static void setLevelToLeds(uint8_t level);
static void driveLeds();
static void lazyDelayMs(uint8_t duration);
static uint16_t unoffsetClip16(uint16_t min, uint16_t range, uint16_t val);

int main(void) {
    // GPIO 設定
    DDRB = 0x1f; // PB0..4 を出力に設定
    PORTB = 0x00; // LOW 出力

    // ADC設定
    ADCSRA = 0b10000111;
    // [7]   ADEN  = 1    ADC有効
    // [6]   ADSC  = 0    ADC停止
    // [5]   ADATE = 0    オートトリガ無効
    // [4]   ADIF  = 0    割り込みなし
    // [3]   ADIE  = 0    割り込み無効
    // [2:0] ADPS  = 0    プリスケーラ: 128 (最大)
    ADMUX = 0x00;
    // [6]   REFS0 = 0    Vref = Vcc
    // [5]   ADLAR = 0    変換結果右寄せ
    // [1:0] MUX   = 0    ADC0 (PB5) 選択
    DIDR0 |= _BV(ADC0D); // ADC0 のデジタル入力無効化

    // 適当に電源電圧が安定するのを待つ
    lazyDelayMs(100);
    //_delay_ms(100);

    // ループ処理開始
    while(true) loop();
    //while(true) { };
}

static void loop(void) {
    uint16_t adcVal = readAdc();

    switch(state) {
    case State::WAIT_OPEN:
        if (ADC_OPEN_MIN - 50 <= adcVal && adcVal < ADC_OPEN_MAX + 50) {
            estimateVcc(adcVal);
            for(int i = 0; i < ADC_OPEN_HISTORY_SIZE; i++) {
                adcOpenHistory[i] = adcVal;
            }
            state = State::OPEN;
        }
        else {
            setLevelToLeds((tickCount & 0x200u) ? LEVEL_MAX : 0);
        }
        break;

    case State::OPEN:
        if (adc2Volt(adcVal) >= VOLT_OPEN_MAX) {
            state = State::MEAS;
        }
        else {
            // 電池電圧や温度変化で基準電圧が変動するので定期的に更新する
            //adcOpenSampleCounter = (adcOpenSampleCounter + 1) & (ADC_OPEN_SAMPLE_INTERVAL - 1);
            adcOpenSampleCounter++;
            //if (adcOpenSampleCounter == 0) {
            if ((adcOpenSampleCounter & (ADC_OPEN_SAMPLE_INTERVAL - 1)) == 0) {
                uint16_t adcOpen = adcVal;
                for(uint8_t i = 0; i < ADC_OPEN_HISTORY_SIZE; i++) {
                    adcOpen = adcOpenHistory[i] < adcOpen ? adcOpenHistory[i] : adcOpen;
                }
                estimateVcc(adcOpen);
                adcOpenHistoryIndex = (adcOpenHistoryIndex + 1) & (ADC_OPEN_HISTORY_SIZE - 1);
                adcOpenHistory[adcOpenHistoryIndex] = adcVal;
                //uint8_t idx = adcOpenSampleCounter / ADC_OPEN_SAMPLE_INTERVAL;
                //uint8_t idx = (adcOpenSampleCounter >> 4) & (ADC_OPEN_HISTORY_SIZE - 1);
                //adcOpenHistory[idx & (ADC_OPEN_HISTORY_SIZE - 1)] = adcVal;
            }
            setVoltageToLeds(VoltRange::BAT_3V0, voltVcc);
        }
        break;

    case State::MEAS:
        meas(adcVal);
        if (range == VoltRange::OPEN) {
            state = State::OPEN;
        }
        break;
    }
    
    driveLeds();
}

//static uint32_t mul16x16(uint32_t a, uint16_t b) __attribute__((noinline));
//static uint32_t mul32x16(uint32_t a, uint16_t b) __attribute__((noinline));
//static uint32_t mul16x16(uint16_t a, uint16_t b) {
//  return (uint32_t)a * b;
//}
//static uint32_t mul32x16(uint32_t a, uint16_t b) {
//  uint16_t ah = (a >> 16) & 0xfffful;
//  uint16_t al = a & 0xfffful;
//  return (mul16x16(ah, b) << 16) + mul16x16(al, b);
//}

// static uint32_t mul32x16(uint32_t a, uint16_t b) __attribute__((noinline));
// static uint32_t mul32x16(uint32_t a, uint16_t b) {
//   uint32_t accum = 0;
//   for (uint8_t ia = 0; ia < 32; ia++) {
//     uint16_t tmpB = b;
//     for (uint8_t ib = 0; ib < 16; ib++) {
//       uint
//     } 
//   }
// }

#define mul32x16(a, b) ((uint32_t)(a) * (b))

static uint16_t readAdc() {
    constexpr uint8_t NUM_SAMPLES = 8;
    uint16_t adcVal = 0;
    PORTB = 0x00;
    //_delay_ms(1);
    lazyDelayMs(1);
    analogRead(); // 読み捨て
    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        adcVal += analogRead();
    }
    return (adcVal + NUM_SAMPLES / 2) / NUM_SAMPLES;
}

static uint16_t analogRead() {
    ADCSRA |= _BV(ADSC); // ADC開始
    // loop_until_bit_is_set(ADCSRA, ADIF); // ADC完了待ち
    while (!(ADCSRA & _BV(ADIF))) {} // ADC完了待ち
    uint8_t lo = ADCL; // Low 側から先に読むこと
    uint8_t hi = ADCH;
    return (((uint16_t)hi) << 8) | lo;
}

// 1N4148W の If * 3 から Vcc 電圧を推定
static void estimateVcc(uint16_t adcVal) {
    constexpr uint16_t ONE = 1ul << 8;
    constexpr uint32_t SLOPE = (uint32_t)ONE * (VOLT_VCC_MAX - VOLT_VCC_MIN) / (ADC_OPEN_MAX - ADC_OPEN_MIN);
    uint16_t tmp = adcVal;
    if (tmp > ADC_OPEN_MAX) tmp = 0;
    else if (tmp < ADC_OPEN_MIN) tmp = (ADC_OPEN_MAX - ADC_OPEN_MIN);
    else tmp = ADC_OPEN_MAX - tmp;
    voltVcc = (uint16_t)(mul32x16(SLOPE, tmp) / ONE) + VOLT_VCC_MIN;

    adc3xVf = adcVal;
    volt3xVf = mul32x16(voltVcc, adc3xVf) / ADC_VCC;
}

static void meas(uint16_t adcVal) {
    constexpr uint8_t VOLT_DIV = 8;
    constexpr uint8_t THRESH_OPEN = (VOLT_OPEN_MAX + VOLT_DIV - 1) / VOLT_DIV;
    constexpr uint8_t THRESH_1V5 = (VOLT_1V5_MAX + VOLT_3V0_MIN + VOLT_DIV) / (2 * VOLT_DIV);
    constexpr uint8_t THRESH_3V0 = (VOLT_3V0_MAX + VOLT_3V7_MIN + VOLT_DIV) / (2 * VOLT_DIV);
    constexpr uint8_t THRESH_3V7 = (VOLT_3V7_MAX + VOLT_9V0_MIN + VOLT_DIV) / (2 * VOLT_DIV);
    uint16_t volt = adc2Volt(adcVal);
    uint8_t voltDiv = volt / VOLT_DIV;
    VoltRange newRange;
    if (voltDiv < THRESH_OPEN) {
        newRange = VoltRange::OPEN;
    }
    else if (voltDiv < THRESH_1V5) {
        newRange = VoltRange::BAT_1V5;
    }
    else if (voltDiv < THRESH_3V0) {
        newRange = VoltRange::BAT_3V0;
    }
    else if (voltDiv < THRESH_3V7) {
        newRange = VoltRange::BAT_3V7;
    }
    else {
        newRange = VoltRange::BAT_9V0;
    }
    range = newRange;
    
    setVoltageToLeds(newRange, volt);
}

static uint16_t adc2Volt(uint16_t adcVal) {
    constexpr uint8_t EXTRA_PREC = (1 << 4);
    uint16_t volt = mul32x16(adcVal, voltVcc * EXTRA_PREC) / ADC_VCC;
    volt = unoffsetClip16(volt3xVf * EXTRA_PREC, (voltVcc - volt3xVf) * EXTRA_PREC, volt);
    return mul32x16(volt, RH + RL) / (RL * EXTRA_PREC);
}

static void setVoltageToLeds(VoltRange range, uint16_t volt) {
    constexpr uint32_t ONE = 1ul << 8;
    uint16_t slope = 0;
    switch(range) {
    case VoltRange::BAT_1V5: 
        slope = ONE * LEVEL_RANGE / (VOLT_1V5_MAX - VOLT_1V5_MIN);
        volt = unoffsetClip16(VOLT_1V5_MIN, VOLT_1V5_MAX - VOLT_1V5_MIN, volt);
        break;

    case VoltRange::BAT_3V0:
        slope = ONE * LEVEL_RANGE / (VOLT_3V0_MAX - VOLT_3V0_MIN);
        volt = unoffsetClip16(VOLT_3V0_MIN, VOLT_3V0_MAX - VOLT_3V0_MIN, volt);
        break;

    case VoltRange::BAT_3V7:
        slope = ONE * LEVEL_RANGE / (VOLT_3V7_MAX - VOLT_3V7_MIN);
        volt = unoffsetClip16(VOLT_3V0_MIN, VOLT_3V7_MAX - VOLT_3V7_MIN, volt);
        break;

    case VoltRange::BAT_9V0:
        slope = ONE * LEVEL_RANGE / (VOLT_9V0_MAX - VOLT_9V0_MIN);
        volt = unoffsetClip16(VOLT_9V0_MIN, VOLT_9V0_MAX - VOLT_9V0_MIN, volt);
        break;
    }
    
    //int16_t level = slope * volt / SLOPE_ONE;
    uint8_t level = mul32x16(slope, volt) / ONE + 1;
    setLevelToLeds(level);
}

static void setLevelToLeds(uint8_t level) {
    int8_t tmp = level;
    for (uint8_t iled = 0; iled < NUM_LEDS; iled++) {
        int8_t val = tmp;
        if (val < 0) {
            val = 0;
        }
        else if (val >= LEVEL_STEP) {
            val = LEVEL_STEP;
        }
        ledValues[iled] = val;
        tmp -= LEVEL_STEP;
    }
}

static void driveLeds() {
    uint8_t phase = ledPhase >> LED_PHASE_STEP_PREC;
    for (uint8_t iled = 0; iled < NUM_LEDS; iled++) {
        if (phase < ledValues[iled]) {
            PORTB |= _BV(iled);
            //digitalWrite(LED_PINS(iled), HIGH);
        }
        lazyDelayMs(LED_DRIVE_INTERVAL_MS);
        //_delay_ms(LED_DRIVE_INTERVAL_MS);
        PORTB &= ~_BV(iled);
        //digitalWrite(LED_PINS(iled), LOW);
        ledPhase = (ledPhase + 1 < LED_PHASE_PERIOD) ? (ledPhase + 1) : 0;
    }
}

// なんとなく 1ms くらいのディレイ
static void lazyDelayMs(uint8_t duration) {
    tickCount += duration;
    while (duration-- > 0) {
        for (uint8_t i = 0; i < 100; i++) {
            asm volatile ("nop");
        }
    }
}

static uint16_t unoffsetClip16(uint16_t min, uint16_t range, uint16_t val) {
    val -= min;
    if (val & 0x8000u) return 0;
    if (val > range) return range;
    return val;
}
