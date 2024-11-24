#define F_CPU 1200000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

static constexpr int NUM_LEDS = 5;
static constexpr uint8_t LED_PIN_MASK = (1 << NUM_LEDS) - 1;

// 電圧の精度
static constexpr int VOLT_PREC = 8;
static constexpr uint16_t VOLT_ONE = 1ul << VOLT_PREC;

// レベル表示の階調
static constexpr uint8_t LEVEL_STEP = 4;
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
static void measureBattery(uint16_t adcVal);
static uint16_t adc2Volt(uint16_t adcVal);
static void setVoltageToLeds(VoltRange range, uint16_t volt);
static void setLevelToLeds(uint8_t level);
static void driveLeds();
static void lazyDelayMs(uint8_t duration);
static uint16_t unoffsetClip16(uint16_t min, uint16_t range, uint16_t val);

#define mul32x16(a, b) ((uint32_t)(a) * (b))

int main(void) {
    // GPIO 設定
    DDRB = LED_PIN_MASK; // PB0..4 を出力に設定
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

    // ループ処理開始
    while(true) loop();
}

static void loop(void) {
    uint16_t adcVal = readAdc();

    switch(state) {
    case State::WAIT_OPEN:
        setLevelToLeds((tickCount & 0x200u) ? LEVEL_MAX : 0);
        if (ADC_OPEN_MIN - 50 <= adcVal && adcVal < ADC_OPEN_MAX + 50) {
            estimateVcc(adcVal);
            for(int i = 0; i < ADC_OPEN_HISTORY_SIZE; i++) {
                adcOpenHistory[i] = adcVal;
            }
            state = State::OPEN;
        }
        break;

    case State::OPEN:
        // 電池電圧や温度変化で基準電圧が変動するので定期的に更新する
        adcOpenSampleCounter++;
        if ((adcOpenSampleCounter & (ADC_OPEN_SAMPLE_INTERVAL - 1)) == 0) {
            uint16_t adcOpen = adcVal;
            for(uint8_t i = 0; i < ADC_OPEN_HISTORY_SIZE; i++) {
                adcOpen = adcOpenHistory[i] < adcOpen ? adcOpenHistory[i] : adcOpen;
            }
            estimateVcc(adcOpen);
            adcOpenHistoryIndex = (adcOpenHistoryIndex + 1) & (ADC_OPEN_HISTORY_SIZE - 1);
            adcOpenHistory[adcOpenHistoryIndex] = adcVal;
        }
        setVoltageToLeds(VoltRange::BAT_3V0, voltVcc);
        
        if (adc2Volt(adcVal) >= VOLT_OPEN_MAX) {
            state = State::MEAS;
        }
        break;

    case State::MEAS:
        measureBattery(adcVal);

        if (range == VoltRange::OPEN) {
            state = State::OPEN;
        }
        break;
    }
    
    driveLeds();
}

static uint16_t readAdc() {
    constexpr uint8_t NUM_SAMPLES = 8;
#if 0
    // ダイナミック点灯を PB4-->PB0 の順にしたので PB4 は十分に長い時間 Low になっているはず
    PORTB = 0x00;
    lazyDelayMs(1);
#endif
    analogRead(); // 読み捨て
    uint16_t adcVal = 0;
    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        adcVal += analogRead();
    }
    return (adcVal + NUM_SAMPLES / 2) / NUM_SAMPLES;
}

static uint16_t analogRead() {
    ADCSRA |= _BV(ADSC); // ADC開始
    loop_until_bit_is_set(ADCSRA, ADIF); // ADC完了待ち
    uint8_t lo = ADCL; // Low 側から先に読むこと
    uint16_t hi = ((uint16_t)ADCH) << 8;
    return hi | lo;
}

// 1N4148W の If * 3 から Vcc 電圧を推定
static void estimateVcc(uint16_t adcVal) {
    constexpr uint16_t ONE = 1u << 8;
    constexpr uint32_t SLOPE = (uint32_t)ONE * (VOLT_VCC_MAX - VOLT_VCC_MIN) / (ADC_OPEN_MAX - ADC_OPEN_MIN);
    uint16_t tmp = adcVal;
    if (tmp > ADC_OPEN_MAX) tmp = 0;
    else if (tmp < ADC_OPEN_MIN) tmp = (ADC_OPEN_MAX - ADC_OPEN_MIN);
    else tmp = ADC_OPEN_MAX - tmp;
    voltVcc = (uint16_t)(mul32x16(SLOPE, tmp) / ONE) + VOLT_VCC_MIN;

    adc3xVf = adcVal;
    volt3xVf = mul32x16(voltVcc, adc3xVf) / ADC_VCC;
}

static void measureBattery(uint16_t adcVal) {
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
    
    uint8_t level = LEVEL_MIN;
    level += mul32x16(slope, volt) / ONE;
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
    uint8_t iled = NUM_LEDS;
    uint8_t ledMask = 1u << (NUM_LEDS - 1);
    while (iled-- != 0) {
        if (phase < ledValues[iled]) {
            PORTB = ledMask;
        }
        ledMask >>= 1;
        lazyDelayMs(LED_DRIVE_INTERVAL_MS);
        PORTB = 0x00;
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
