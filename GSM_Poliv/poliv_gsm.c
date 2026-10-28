/*
 * Система полива с GSM-управлением — версия 3
 * ATmega328P, внутренний RC-генератор 8 МГц
 *
 * ФЬЮЗЫ (ставятся один раз, перед первой прошивкой):
 *   lfuse = 0xE2  — внутренний RC 8 МГц, без деления на 8 (CKDIV8 выключен).
 *                   С заводским 0x62 чип работает на 1 МГц, и UART не работает.
 *   hfuse = 0xD1  — EESAVE включён: при перепрошивке EEPROM (админ, расписания,
 *                   USSD-код, калибровка) НЕ стирается. Заводское 0xD9 — стирается.
 *   efuse = 0xFD  — BOD 2.7 В: при просадке питания контроллер уходит в сброс,
 *                   а не портит EEPROM. (Старые avrdude пишут это как 0x05.)
 *
 *   Прочитать:  avrdude -c usbasp -p m328p -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h
 *   Записать:   avrdude -c usbasp -p m328p -U lfuse:w:0xE2:m -U hfuse:w:0xD1:m -U efuse:w:0xFD:m
 *   Прошивка:   avrdude -c usbasp -p m328p -U flash:w:poliv.hex
 *
 *   В AVR Studio 4 (вкладка Fuses):
 *     CKDIV8 — снять;  EESAVE — поставить;  BODLEVEL — Brown-out at VCC=2.7 V;
 *     SUT_CKSEL — Int. RC Osc. 8 MHz; Start-up time PWRDWN/RESET: 6 CK/14 CK + 65 ms
 *     SPIEN — не трогать (без него чип не прошить через USBasp).
 *
 * GSM-модуль M590E всегда включён и в сети (сон без провода DTR не работает).
 * Контроллер спит (Power-down) и просыпается:
 *   - по сигналу модуля — прерывание PCINT на PD2 (RI, срабатывает на звонок)
 *     и на PD0 (RX: при новой SMS модуль сам присылает в UART "+CMTI: ...");
 *   - раз в час — страховочная проверка SIM (SMS_POLL_MIN);
 *   - каждые ~8 с от WDT — часы, таймеры реле, поплавки, расписание.
 * Время:
 *   1) AT+CLTS=1 — сеть присылает время (*PSUTTZ), плюс чтение часов модуля AT+CCLK?;
 *   2) метка каждой принятой SMS.
 *
 * Пины:
 *   Реле 1..4 : PD4, PB1, PB2, PB3 (включаются LOW)
 *   Реле 5    : PC4 (включается HIGH, см. REL5_ACTIVE_HIGH)
 *   Серво     : PD5 (программный ШИМ)
 *   Поплавки  : PD7, PB0, PC2, PC3 (замыкают на GND)
 *   DHT11     : PD6
 *   Почва     : ADC0, ADC1      Батарея: ADC5 (47k к +12 В / 20k к GND)
 *   GSM       : PD0/PD1 UART 9600, PD2 <- I (RI), PD3 -> K (PWRKEY)
 *
 * SMS-команды (регистр не важен):
 *  С любого номера:
 *   status              — статус системы (ответ одной SMS)
 *   1-on-20 / 5-on-10.5 — включить реле на N минут
 *   1-off               — выключить реле
 *   6-angle-90          — повернуть серво (0..180)
 *  Только админ:
 *   admin               — стать админом (если админ ещё не задан)
 *   admin +79991234567  — передать права другому номеру
 *   1-auto-18-20        — автополив каждый день в 18:00 на 20 мин
 *   1-auto-18:30-20     — то же в 18:30
 *   1-auto / 1-auto-off — включить / выключить сохранённое расписание
 *   balance *100#       — запомнить USSD-код и запросить баланс
 *   balance             — запросить баланс
 *   adc                 — АЦП, RI, поплавки, время и его источник
 *                         (D — отчёт о доставке, N — сеть, S — SMS, - — нет)
 *   s1-dry / s1-wet     — калибровка датчика почвы 1: запомнить текущее показание
 *                         как "сухо" (датчик в воздухе) / "мокро" (в воде)
 *   s2-dry / s2-wet     — то же для датчика 2 (можно задать число: s1-dry-600)
 *   batt-12.14          — калибровка АКБ: реальное напряжение с мультиметра
 *
 * В одной SMS можно несколько команд через ';' или с новой строки:
 *   1-auto-18-20; 2-auto-19-10; status
 * Отвечает только на status, balance, adc. После включения шлёт админу
 * "Restart (причина). ..." — ответ любой SMS установит часы.
 * Любая неправильная SMS пересылается админу: "From <номер>: <текст>".
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/* ========================= НАСТРОЙКИ ========================= */

#define WDT_TICK_MS        8000   /* период WDT, мс; если часы спешат/отстают — поправить */

#define BATT_CHARGE_X10    130    /* выше 13.0 В — в статусе "crg"               */
#define BATT_PUMP_MIN_X10  115    /* ниже 11.5 В реле не включаются             */
#define BATT_K_DEFAULT     1637   /* V*10 = raw * K / 10000 (делитель 47k/20k); SMS batt-12.6 */

#define SOIL_DRY_RAW       600    /* по умолчанию: датчик в воздухе (SMS s1-dry)     */
#define SOIL_WET_RAW       280    /* по умолчанию: датчик в воде    (SMS s1-wet)     */

#define SERVO_MIN_US       500    /* импульс для 0°   */
#define SERVO_MAX_US       2500   /* импульс для 180° */

#define MODULE_SLEEP       0      /* 1 — сон модуля (AT+ENPWRSAVE=1). ПРОВЕРЕНО: без провода
                                   *     DTR модуль уходит из сети ("абонент недоступен").
                                   *     Включать только после подключения DTR. */
#define SMS_POLL_MIN       60     /* страховочная проверка SIM раз в N минут     */
#define SLEEP_POLL_MIN     60     /* то же при MODULE_SLEEP 1                    */
#define GSM_RETRY_MIN      5      /* если модуль не запустился — повтор через N мин */
#define NET_WAIT_TRIES     30     /* попыток проверки регистрации (по ~2 с)      */
#define START_MAX_S        180    /* лимит на запуск модуля, с                   */
#define CHECK_MAX_S        90     /* лимит на проверку SIM, с                    */

#define REL5_ACTIVE_HIGH   1      /* реле 5 включается высоким уровнем           */
#define RI_PULLUP          1      /* внутренняя подтяжка RI (у модуля открытый сток) */
#define RI_WAKE            1      /* 1 — будить по RI (звонок); 0 — только по RX (SMS),
                                   *     если RI шумит и будит контроллер впустую */
#define READ_UCS2          1      /* русские SMS -> транслит (0 — будут "????")  */
#define FORWARD_BAD_SMS    1      /* пересылать админу неправильные SMS          */
#define RESTART_SMS        1      /* SMS админу после включения/перезагрузки     */
#define RESTART_TIME_REPORT 1     /* 1 — у этой SMS запросить отчёт о доставке и
                                   * взять из него время (если модуль умеет);
                                   * если перестанут работать SMS — поставить 0   */

#define DEBUG_LEDS         0      /* ТЕСТ: индикация на светодиодах реле 1–4.
                                   * Снимите перемычку JD-VCC! Реле 1–4 при этом
                                   * не управляются. После теста — 0.            */

/* Реле 1..4 блокируются, когда стабильное состояние поплавка равно этому
 * значению (1 = поплавок замкнут на GND). */
static const uint8_t float_block_state[4] = { 0, 1, 0, 1 };

/* ========================= ПИНЫ ========================= */

#define REL1_PIN    PD4
#define REL2_PIN    PB1
#define REL3_PIN    PB2
#define REL4_PIN    PB3
#define REL5_PIN    PC4
#define SERVO_PIN   PD5
#define FLOAT1_PIN  PD7
#define FLOAT2_PIN  PB0
#define FLOAT3_PIN  PC2
#define FLOAT4_PIN  PC3
#define DHT_PIN     PD6
#define SOIL1_CH    0
#define SOIL2_CH    1
#define BATT_CH     5
#define GSM_RI_PIN  PD2
#define GSM_PWR_PIN PD3

/* ========================= EEPROM ========================= */

#define EE_SCHED_ADDR  1     /* 5 реле x 5 байт */
#define EE_USSD_ADDR   26    /* 20 байт */
#define EE_SERVO_ADDR  46
#define EE_ADMIN_ADDR  47    /* 20 байт */
#define EE_STR_LEN     20
#define EE_CAL_ADDR    100   /* s1 сухо, s1 мокро, s2 сухо, s2 мокро, K батареи — по 2 байта */

/* ========================= ДАННЫЕ ========================= */

typedef struct {
    uint8_t  on;
    uint16_t timer;          /* осталось, с */
    uint8_t  last_h, last_m; /* 0xFF = не запускалось */
    uint16_t last_dur;       /* с */
    uint8_t  auto_en, auto_h, auto_m;
    uint16_t auto_dur;       /* с */
} Relay;

static volatile Relay    relays[5];
static volatile uint8_t  hh, mm, ss, time_valid;
static volatile uint8_t  minute_flag;
static volatile uint8_t  float_hist[4];
static volatile uint8_t  batt_x10;
static volatile uint32_t uptime_sec;
static volatile uint8_t  ri_flag;

static char     admin[EE_STR_LEN];
static char     ussd[EE_STR_LEN];
static uint8_t  gsm_ok;              /* модуль запущен и в сети */
static uint8_t  gsm_fail;            /* подряд неудачных проверок */
static uint32_t last_sms_time;       /* метка последней SMS, по которой ставили время */
static const char *reply_to = "";
static uint8_t  op_active;
static uint32_t op_deadline;
static uint8_t  dbg_mask;
static uint8_t  servo_angle   = 90;
static uint8_t  servo_pending = 0xFF;
static uint16_t soil_cal[2][2];      /* [датчик][0 — сухо, 1 — мокро], сырые значения АЦП */
static uint16_t batt_k = BATT_K_DEFAULT;

static char line[128];
static char big[161];

typedef struct { uint8_t idx; char num[EE_STR_LEN]; char text[110]; char ts[24]; } Sms;
static Sms inbox;

static uint8_t gsm_send_sms(const char *num, const char *text);
static void    process_inbox(void);

/* ========================= РЕЛЕ И ПОПЛАВКИ ========================= */

static void relay_hw(uint8_t i, uint8_t on) {
    if (DEBUG_LEDS && i < 4) return;               /* пины 1–4 заняты индикацией */
    switch (i) {
    case 0: if (on) PORTD &= ~(1 << REL1_PIN); else PORTD |= (1 << REL1_PIN); break;
    case 1: if (on) PORTB &= ~(1 << REL2_PIN); else PORTB |= (1 << REL2_PIN); break;
    case 2: if (on) PORTB &= ~(1 << REL3_PIN); else PORTB |= (1 << REL3_PIN); break;
    case 3: if (on) PORTB &= ~(1 << REL4_PIN); else PORTB |= (1 << REL4_PIN); break;
    case 4:
        if (on == REL5_ACTIVE_HIGH) PORTC |= (1 << REL5_PIN); else PORTC &= ~(1 << REL5_PIN);
        break;
    }
}

static uint8_t float_raw(uint8_t i) {
    switch (i) {
    case 0: return !(PIND & (1 << FLOAT1_PIN));
    case 1: return !(PINB & (1 << FLOAT2_PIN));
    case 2: return !(PINC & (1 << FLOAT3_PIN));
    case 3: return !(PINC & (1 << FLOAT4_PIN));
    }
    return 0;
}

static uint8_t float_stable(uint8_t i) {
    uint8_t h = float_hist[i];
    return ((h & 1) + ((h >> 1) & 1) + ((h >> 2) & 1)) >= 2;
}

static uint8_t relay_blocked(uint8_t i) {
    return (i < 4) && (float_stable(i) == float_block_state[i]);
}

/* вызывать из ISR или при запрещённых прерываниях */
static void relay_stop(uint8_t i) {
    relay_hw(i, 0);
    relays[i].on = 0;
    relays[i].timer = 0;
}

/* вызывать из ISR или при запрещённых прерываниях */
static void relay_start(uint8_t i, uint16_t sec) {
    if (!sec || relay_blocked(i)) return;
    if (batt_x10 >= 50 && batt_x10 < BATT_PUMP_MIN_X10) return;   /* <5 В = нет измерения */
    relay_hw(i, 1);
    relays[i].on = 1;
    relays[i].timer = sec;
    relays[i].last_dur = sec;
    relays[i].last_h = time_valid ? hh : 0xFF;
    relays[i].last_m = time_valid ? mm : 0xFF;
}

/* ========================= ПРЕРЫВАНИЯ ========================= */

/* WDT ~8 с: часы, поплавки, таймеры реле, расписание */
ISR(WDT_vect) {
    static uint16_t ms_acc = 0;
    uint8_t secs = 0, i;

    ms_acc += WDT_TICK_MS;
    while (ms_acc >= 1000) { ms_acc -= 1000; secs++; }
    uptime_sec += secs;

    ss += secs;
    while (ss >= 60) {
        ss -= 60;
        if (++mm >= 60) { mm = 0; if (++hh >= 24) hh = 0; }
        minute_flag = 1;
        if (time_valid) {
            for (i = 0; i < 5; i++)
                if (relays[i].auto_en && !relays[i].on &&
                    relays[i].auto_h == hh && relays[i].auto_m == mm)
                    relay_start(i, relays[i].auto_dur);
        }
    }

    for (i = 0; i < 4; i++)
        float_hist[i] = ((float_hist[i] << 1) | float_raw(i)) & 0x07;

    for (i = 0; i < 5; i++) {
        if (!relays[i].on) continue;
        if (relays[i].timer > secs) relays[i].timer -= secs;
        else { relay_stop(i); continue; }
        if (relay_blocked(i)) relay_stop(i);
    }
}

/* RI (PD2) или начало строки от модуля на RX (PD0). Флаг PCINT защёлкивается
 * аппаратно, поэтому даже короткий импульс будит контроллер из Power-down. */
ISR(PCINT2_vect) {
    ri_flag = 1;
}

/* ========================= ВРЕМЯ ИЗ SMS ========================= */

static void set_time(uint8_t h, uint8_t m, uint8_t s) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { hh = h; mm = m; ss = s; time_valid = 1; }
}

static uint8_t two_dig(const char *p) {
    if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9') return 0xFF;
    return (p[0] - '0') * 10 + (p[1] - '0');
}

static const uint16_t cum_days[12] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

/* метка SMS "yy/MM/dd,hh:mm:ss+zz". Более старые SMS (например, пролежавшие
 * на SIM) время не сбивают — принимаем только метку новее предыдущей. */
static char time_src = '-';          /* чем установлено время после старта: D/N/S */
static uint8_t net_seen;             /* время от сети приходило хоть раз после старта */
static uint8_t clts_ok;              /* модуль принял AT+CLTS=1 */

/* src: 'S' — метка SMS, 'N' — часы модуля (AT+CCLK, время от оператора) */
static uint8_t sms_time_update(const char *ts, char src) {
    uint8_t y, mo, d, h, m, s;
    uint32_t days, t;
    if (strlen(ts) < 17) return 0;
    y = two_dig(ts); mo = two_dig(ts + 3); d = two_dig(ts + 6);
    h = two_dig(ts + 9); m = two_dig(ts + 12); s = two_dig(ts + 15);
    if (y < 24 || y > 60 || mo < 1 || mo > 12 || d < 1 || d > 31 || h > 23 || m > 59 || s > 59) return 0;
    y -= 24;
    days = (uint32_t)y * 365 + (y + 3) / 4 + cum_days[mo - 1] +
           ((mo > 2 && (y % 4) == 0) ? 1 : 0) + d - 1;
    t = days * 86400UL + (uint32_t)h * 3600UL + (uint16_t)m * 60 + s;
    if (time_valid && t < last_sms_time) return 1;   /* метка старая — время не трогаем */
    last_sms_time = t;
    if (!time_valid) time_src = src;
    set_time(h, m, s);
    return 1;
}

/* ========================= АЦП, ДАТЧИКИ ========================= */

static uint16_t adc_read(uint8_t ch) {
    uint16_t v;
    ADMUX  = (1 << REFS0) | (ch & 0x0F);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);   /* 125 кГц */
    _delay_us(100);
    ADCSRA |= (1 << ADSC); while (ADCSRA & (1 << ADSC));  /* первое — в мусор */
    ADCSRA |= (1 << ADSC); while (ADCSRA & (1 << ADSC));
    v = ADC;
    ADCSRA = 0;
    return v;
}

/* сумма 8 измерений АКБ — меньше шума */
static uint16_t batt_raw_x8(void) {
    uint16_t sum = 0;
    uint8_t i;
    for (i = 0; i < 8; i++) sum += adc_read(BATT_CH);
    return sum;
}

static uint8_t measure_batt_x10(void) {
    uint32_t v = ((uint32_t)batt_raw_x8() * batt_k + 40000UL) / 80000UL;   /* с округлением */
    return v > 255 ? 255 : (uint8_t)v;
}

/* влажность почвы 0..100% между точками "сухо" и "мокро".
 * Работает для любого датчика: ёмкостного (сухо > мокро) и резистивного (наоборот). */
static uint8_t soil_pct(uint8_t i) {
    int32_t raw = adc_read(i == 0 ? SOIL1_CH : SOIL2_CH);
    int32_t dry = soil_cal[i][0], wet = soil_cal[i][1];
    int32_t p = (dry - raw) * 100 / (dry - wet);
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    return (uint8_t)p;
}

static uint8_t dht_wait(uint8_t level) {
    uint8_t t = 0;
    while (((PIND >> DHT_PIN) & 1) != level) {
        if (++t == 0) return 0;   /* ~400 мкс — датчик не отвечает */
        _delay_us(1);
    }
    return 1;
}

static uint8_t read_dht11(uint8_t *temp, uint8_t *hum) {
    uint8_t d[5] = { 0 }, i, j, ok = 0, sreg;

    DDRD  |=  (1 << DHT_PIN);
    PORTD &= ~(1 << DHT_PIN);
    _delay_ms(20);

    sreg = SREG; cli();
    PORTD |=  (1 << DHT_PIN);
    DDRD  &= ~(1 << DHT_PIN);
    _delay_us(30);
    if (!dht_wait(0) || !dht_wait(1) || !dht_wait(0)) goto out;

    for (i = 0; i < 5; i++) {
        uint8_t b = 0;
        for (j = 0; j < 8; j++) {
            if (!dht_wait(1)) goto out;
            _delay_us(35);
            b <<= 1;
            if (PIND & (1 << DHT_PIN)) {
                b |= 1;
                if (!dht_wait(0)) goto out;
            }
        }
        d[i] = b;
    }
    ok = (d[4] == (uint8_t)(d[0] + d[1] + d[2] + d[3]));
out:
    SREG = sreg;
    if (ok) { *hum = d[0]; *temp = d[2]; }
    return ok;
}

/* ========================= СЕРВО ========================= */

static void servo_move(uint8_t angle) {
    uint16_t us, n;
    uint8_t k, sreg;
    if (angle > 180) angle = 180;
    us = SERVO_MIN_US + (uint16_t)(((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle) / 180);

    for (k = 0; k < 50; k++) {              /* ~1 с импульсов, потом серво отпускается */
        sreg = SREG; cli();
        PORTD |= (1 << SERVO_PIN);
        for (n = us / 10; n; n--) _delay_us(10);
        PORTD &= ~(1 << SERVO_PIN);
        SREG = sreg;
        _delay_ms(18);
    }
    servo_angle = angle;
    eeprom_update_byte((uint8_t *)EE_SERVO_ADDR, angle);
}

/* ========================= EEPROM ========================= */

static void save_schedule(uint8_t i) {
    uint16_t a = EE_SCHED_ADDR + i * 5;
    eeprom_update_byte((uint8_t *)a,        relays[i].auto_en);
    eeprom_update_byte((uint8_t *)(a + 1),  relays[i].auto_h);
    eeprom_update_byte((uint8_t *)(a + 2),  relays[i].auto_m);
    eeprom_update_word((uint16_t *)(a + 3), relays[i].auto_dur);
}

static void ee_save_str(uint16_t addr, const char *s) {
    uint8_t i;
    for (i = 0; i < EE_STR_LEN; i++) {
        eeprom_update_byte((uint8_t *)(addr + i), s[i]);
        if (!s[i]) break;
    }
}

static void ee_load_str(uint16_t addr, char *s) {
    uint8_t i;
    for (i = 0; i < EE_STR_LEN - 1; i++) {
        s[i] = eeprom_read_byte((const uint8_t *)(addr + i));
        if (!s[i]) break;
    }
    s[EE_STR_LEN - 1] = 0;
}

/* строка из EEPROM годится: первый символ из first, дальше печатные символы */
static uint8_t str_valid(const char *s, const char *first) {
    if (!s[0] || !strchr(first, s[0])) return 0;
    for (; *s; s++) if (*s < 32 || *s > 126) return 0;
    return 1;
}

/* Каждое поле проверяется отдельно, ничего не стирается целиком:
 * чистая EEPROM (0xFF) даёт значения по умолчанию, а если испортится
 * один байт, остальные настройки сохранятся. */
static void settings_load(void) {
    uint8_t i;

    for (i = 0; i < 5; i++) {
        uint16_t a = EE_SCHED_ADDR + i * 5;
        relays[i].auto_en  = eeprom_read_byte((const uint8_t *)a);
        relays[i].auto_h   = eeprom_read_byte((const uint8_t *)(a + 1));
        relays[i].auto_m   = eeprom_read_byte((const uint8_t *)(a + 2));
        relays[i].auto_dur = eeprom_read_word((const uint16_t *)(a + 3));
        if (relays[i].auto_h > 23 || relays[i].auto_m > 59 || !relays[i].auto_dur)
            relays[i].auto_en = 0;
        relays[i].last_h = 0xFF;
        relays[i].last_m = 0xFF;
    }
    ee_load_str(EE_USSD_ADDR, ussd);
    ee_load_str(EE_ADMIN_ADDR, admin);
    if (!str_valid(admin, "+0123456789")) admin[0] = 0;
    if (!str_valid(ussd, "*#")) ussd[0] = 0;

    servo_angle = eeprom_read_byte((const uint8_t *)EE_SERVO_ADDR);
    if (servo_angle > 180) servo_angle = 90;

    /* калибровка; пустая EEPROM (0xFFFF) или мусор — значения по умолчанию */
    for (i = 0; i < 2; i++) {
        uint16_t d = eeprom_read_word((const uint16_t *)(EE_CAL_ADDR + i * 4));
        uint16_t w = eeprom_read_word((const uint16_t *)(EE_CAL_ADDR + i * 4 + 2));
        if (d > 1023 || w > 1023 || (d > w ? d - w : w - d) < 30) { d = SOIL_DRY_RAW; w = SOIL_WET_RAW; }
        soil_cal[i][0] = d;
        soil_cal[i][1] = w;
    }
    batt_k = eeprom_read_word((const uint16_t *)(EE_CAL_ADDR + 8));
    if (batt_k < 800 || batt_k > 3300) batt_k = BATT_K_DEFAULT;   /* ±2 раза от расчётного */
}

/* ========================= ДИАГНОСТИКА НА СВЕТОДИОДАХ РЕЛЕ ========================= */

/* бит0 — работа с модулем, бит1 — модуль ответил, бит2 — в сети, бит3 — всё прошло */
static void dbg_leds(uint8_t mask) {
    if (!DEBUG_LEDS) return;
    if (mask & 1) PORTD &= ~(1 << REL1_PIN); else PORTD |= (1 << REL1_PIN);
    if (mask & 2) PORTB &= ~(1 << REL2_PIN); else PORTB |= (1 << REL2_PIN);
    if (mask & 4) PORTB &= ~(1 << REL3_PIN); else PORTB |= (1 << REL3_PIN);
    if (mask & 8) PORTB &= ~(1 << REL4_PIN); else PORTB |= (1 << REL4_PIN);
}

static void dbg_stage(uint8_t bit) { dbg_mask |= bit; dbg_leds(dbg_mask); }

static void dbg_flash(uint16_t ms) {       /* вспышка LED 4: 200 мс — SMS прочитана, 1 с — SMS ушла */
    if (!DEBUG_LEDS) return;
    dbg_leds(dbg_mask | 8);
    while (ms--) _delay_ms(1);
    dbg_leds(dbg_mask);
}

static void dbg_boot(void) {               /* бегущий огонь = контроллер стартовал */
    uint8_t k;
    if (!DEBUG_LEDS) return;
    for (k = 0; k < 8; k++) { dbg_leds(1 << (k & 3)); _delay_ms(150); }
    dbg_leds(0);
}

static void dbg_result(void) {             /* итог последней работы с модулем, раз в минуту на 2 с */
    if (!DEBUG_LEDS || !dbg_mask) return;
    dbg_leds(dbg_mask); _delay_ms(2000); dbg_leds(0);
}

/* ========================= ОГРАНИЧЕНИЕ ВРЕМЕНИ ОПЕРАЦИЙ ========================= */

static uint32_t uptime(void) {
    uint32_t u;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { u = uptime_sec; }
    return u;
}

static void op_begin(uint16_t max_s) { op_deadline = uptime() + max_s; op_active = 1; }
static void op_end(void) { op_active = 0; }
static uint8_t op_expired(void) { return op_active && uptime() > op_deadline; }

/* ========================= UART ========================= */

#define RX_SIZE 64
static volatile uint8_t rx_buf[RX_SIZE];
static volatile uint8_t rx_head, rx_tail;

ISR(USART_RX_vect) {
    uint8_t c = UDR0;
    uint8_t n = (rx_head + 1) & (RX_SIZE - 1);
    if (n != rx_tail) { rx_buf[rx_head] = c; rx_head = n; }
}

static void uart_on(void) {
    PRR &= ~(1 << PRUSART0);
    UBRR0H = 0;
    UBRR0L = 51;                                   /* 9600 при 8 МГц */
    UCSR0A = 0;
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    rx_head = rx_tail = 0;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

/* модуль включён, контроллер спит: UART выключен, TX держим в HIGH (линия в покое) */
static void uart_idle(void) {
    UCSR0B = 0;
    PORTD |=  (1 << PD1);
    DDRD  |=  (1 << PD1);
    DDRD  &= ~(1 << PD0);
    PORTD &= ~(1 << PD0);
    PRR |= (1 << PRUSART0);
}

/* модуль не отвечает: TX в Hi-Z, чтобы не подпитывать его через линию */
static void uart_off(void) {
    UCSR0B = 0;
    DDRD  &= ~((1 << PD0) | (1 << PD1));
    PORTD &= ~((1 << PD0) | (1 << PD1));
    PRR |= (1 << PRUSART0);
}

static int16_t uart_getc(void) {
    uint8_t c;
    if (rx_head == rx_tail) return -1;
    c = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) & (RX_SIZE - 1);
    return c;
}

static void uart_flush(void) { rx_tail = rx_head; }

static void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

/* *PSUTTZ: 2026,7,24,11,5,0,"+12",0 — время UTC и пояс в четвертях часа */
static void psuttz_parse(const char *p) {
    int16_t f[6], tz = 0;
    int32_t sod;
    uint8_t i;
    p = strchr(p, ':');
    if (!p) return;
    for (i = 0; i < 6; i++) {
        p++;
        while (*p == ' ') p++;
        f[i] = atoi(p);
        p = strchr(p, ',');
        if (!p) return;
    }
    if (f[3] > 23 || f[4] > 59 || f[5] > 59 || f[0] < 2024) return;
    p = strchr(p, '"');
    if (p) tz = atoi(p + 1);                          /* "+12" -> 12 четвертей = +3 ч */
    sod = (int32_t)f[3] * 3600 + f[4] * 60 + f[5] + (int32_t)tz * 900;
    while (sod < 0) sod += 86400L;
    sod %= 86400L;
    if (!time_valid) time_src = 'N';
    net_seen = 1;
    set_time(sod / 3600, (sod / 60) % 60, sod % 60);
}

/* строка от модуля; timeout — мс тишины. Хвост длинной строки отбрасывается. */
static uint8_t read_line(char *buf, uint8_t max, uint16_t timeout_ms) {
    uint8_t n = 0;
    uint16_t t = 0, guard = 0;
    for (;;) {
        int16_t c = uart_getc();
        if (c < 0) {
            if (t >= timeout_ms || op_expired()) break;
            _delay_ms(1); t++;
            continue;
        }
        t = 0;
        if (++guard > 2000) break;
        if (c == '\r' || c == '\n') { if (n) break; continue; }
        if (n < max - 1) buf[n++] = (char)c;
    }
    buf[n] = 0;
    if (n > 8 && !strncmp(buf, "*PSUTTZ:", 8)) psuttz_parse(buf);   /* время от сети пришло само */
    return n;
}

static uint8_t wait_char(char ch, uint16_t timeout_ms) {
    while (timeout_ms && !op_expired()) {
        int16_t c = uart_getc();
        if (c < 0) { _delay_ms(1); timeout_ms--; }
        else if (c == ch) return 1;
    }
    return 0;
}

static uint8_t wait_response(const char *expect, uint16_t timeout_ms) {
    while (read_line(line, sizeof line, timeout_ms)) {
        if (strstr(line, expect)) return 1;
        if (strstr(line, "ERROR")) return 0;
    }
    return 0;
}

static void send_cmd(const char *cmd) {
    uart_flush();
    uart_puts(cmd);
    uart_putc('\r');
}

static uint8_t at_cmd(const char *cmd, const char *expect, uint16_t timeout_ms) {
    send_cmd(cmd);
    return wait_response(expect, timeout_ms);
}

/* n-я строка в кавычках (с нуля) */
static uint8_t get_quoted(const char *s, uint8_t n, char *out, uint8_t max) {
    for (;;) {
        const char *a = strchr(s, '"'), *b;
        if (!a) return 0;
        b = strchr(a + 1, '"');
        if (!b) return 0;
        if (n == 0) {
            uint8_t len = b - a - 1;
            if (len >= max) len = max - 1;
            memcpy(out, a + 1, len);
            out[len] = 0;
            return 1;
        }
        n--;
        s = b + 1;
    }
}

/* ========================= UCS2 И ТРАНСЛИТ ========================= */

static uint8_t hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0xFF;
}

static uint8_t is_ucs2(const char *s) {
    uint8_t len = strlen(s), i;
    if (len < 8 || (len & 3)) return 0;
    for (i = 0; i < len; i++) if (hexval(s[i]) == 0xFF) return 0;
    return 1;
}

static const char tr_tab[32][4] PROGMEM = {
    "a", "b", "v", "g", "d", "e", "zh", "z", "i", "y", "k", "l", "m", "n", "o", "p",
    "r", "s", "t", "u", "f", "kh", "ts", "ch", "sh", "sch", "", "y", "", "e", "yu", "ya"
};

/* символы, которых нет в алфавите GSM или которые он искажает */
static char gsm_safe(char c) {
    switch (c) {
    case '@': return 'a';
    case '$': return 'S';
    case '_': return '-';
    case '{': case '[': return '(';
    case '}': case ']': return ')';
    case '\\': case '|': case '~': case '^': case '`': return '/';
    }
    return (c >= 32 && c < 127) ? c : ' ';
}

static void put_uc(uint16_t code, char *out, uint8_t *n, uint8_t max) {
    char buf[4];
    const char *t = buf;
    buf[1] = 0;
    if (code == 0x0A || code == 0x0D) buf[0] = ';';    /* перенос строки = разделитель команд */
    else if (code < 128) buf[0] = gsm_safe((char)code);
    else if (code >= 0x410 && code <= 0x44F) {
        strcpy_P(buf, tr_tab[(code - 0x410) & 31]);
        if (code < 0x430 && buf[0]) buf[0] -= 32;          /* заглавная */
    }
    else if (code == 0x401) buf[0] = 'E';
    else if (code == 0x451) buf[0] = 'e';
    else if (code == 0x2116) buf[0] = 'N';                 /* № */
    else buf[0] = ' ';
    while (*t && *n < max - 1) out[(*n)++] = *t++;
    out[*n] = 0;
}

/* UCS2-hex -> латиница (или как есть, если это не hex) */
static void ucs2_field(const char *src, char *out, uint8_t max) {
    uint8_t n = 0;
    out[0] = 0;
    if (!is_ucs2(src)) {
        while (*src && n < max - 1) out[n++] = gsm_safe(*src++);
        out[n] = 0;
        return;
    }
    while (src[0] && src[1] && src[2] && src[3]) {
        uint16_t code = ((uint16_t)hexval(src[0]) << 12) | ((uint16_t)hexval(src[1]) << 8) |
                        (hexval(src[2]) << 4) | hexval(src[3]);
        put_uc(code, out, &n, max);
        src += 4;
    }
}

/* текст SMS: UCS2 декодируется на лету (длинный текст не влезает в буфер целиком) */
static void read_sms_text(char *out, uint8_t max, uint16_t timeout_ms) {
    uint8_t n = 0, raw = 0, hex_ok = 1, q = 0;
    uint16_t code = 0, t = 0, guard = 0;
    out[0] = 0;
    for (;;) {
        int16_t c = uart_getc();
        if (c < 0) {
            if (t >= timeout_ms || op_expired()) break;
            _delay_ms(1); t++;
            continue;
        }
        t = 0;
        if (++guard > 2000) break;
        if (c == '\r' || c == '\n') { if (raw) break; continue; }
        if (raw < sizeof big - 1) big[raw++] = (char)c;
        if (hexval((char)c) == 0xFF) hex_ok = 0;
        else if (hex_ok) {
            code = (code << 4) | hexval((char)c);
            if (++q == 4) { put_uc(code, out, &n, max); q = 0; code = 0; }
        }
    }
    big[raw] = 0;
    if (!hex_ok || q || raw < 4) ucs2_field(big, out, max);   /* обычный текст */
}

/* ========================= GSM ========================= */

/* K-пин как "открытый сток": только тянем к земле или отпускаем */
static void gsm_pwrkey_pulse(void) {
    PORTD &= ~(1 << GSM_PWR_PIN);
    DDRD  |=  (1 << GSM_PWR_PIN);
    _delay_ms(1000);
    DDRD  &= ~(1 << GSM_PWR_PIN);
}

/* пробуждение от модуля: PCINT18 (PD2, RI) и PCINT16 (PD0, RX UART).
 * Включается только пока модуль работает и контроллер с ним не разговаривает. */
static void ri_enable(uint8_t on) {
    if (on) {
        if (RI_PULLUP) PORTD |= (1 << GSM_RI_PIN);
        _delay_ms(50);                                 /* дать модулю договорить */
        PCMSK2 = (RI_WAKE ? (1 << PCINT18) : 0) | (1 << PCINT16);
        PCIFR  = (1 << PCIF2);
        PCICR |= (1 << PCIE2);
    } else {
        PCICR &= ~(1 << PCIE2);
        PORTD &= ~(1 << GSM_RI_PIN);
    }
}

/* в режиме UCS2 модуль ждёт и параметр в UCS2 — пробуем оба варианта */
static void set_charset_gsm(void) {
    if (!at_cmd("AT+CSCS=\"GSM\"", "OK", 2000))
        at_cmd("AT+CSCS=\"00470053004D\"", "OK", 2000);
}

static uint8_t gsm_registered(void) {
    uint8_t reg = 0;
    send_cmd("AT+CREG?");
    while (read_line(line, sizeof line, 1000)) {
        char *p = strstr(line, "+CREG:");
        if (p) {
            p = strchr(p, ',');
            if (p) { p++; while (*p == ' ') p++; if (*p == '1' || *p == '5') reg = 1; }
        } else if (!strcmp(line, "OK") || strstr(line, "ERROR")) break;
    }
    return reg;
}

/* время из часов модуля. Годится, только если модуль получил его от оператора
 * (год >= 2024); у ненастроенных часов год 2000/2004 — такое игнорируется. */
static void gsm_clock_time(void) {
    char q[24];
    send_cmd("AT+CCLK?");
    while (read_line(line, sizeof line, 1000)) {
        if (strstr(line, "+CCLK:")) {
            if (get_quoted(line, 0, q, sizeof q) && sms_time_update(q, 'N')) net_seen = 1;
        } else if (!strcmp(line, "OK") || strstr(line, "ERROR")) break;
    }
}

/* запуск модуля: включить (если молчит), дождаться сети, настроить, прочитать SIM */
static uint8_t gsm_start(void) {
    uint8_t i, alive = 0, ok = 0;

    op_begin(START_MAX_S);
    dbg_mask = 0;
    dbg_stage(1);
    uart_on();

    for (i = 0; i < 5 && !alive; i++) alive = at_cmd("AT", "OK", 1000);   /* уже включён? */
    if (!alive) {
        gsm_pwrkey_pulse();
        _delay_ms(3000);
        for (i = 0; i < 15 && !alive; i++) alive = at_cmd("AT", "OK", 1000);
    }

    if (alive) {
        dbg_stage(2);
        at_cmd("ATE0", "OK", 1000);
        at_cmd("AT+ENPWRSAVE=0", "OK", 1000);          /* во время настройки не спать */
        clts_ok = at_cmd("AT+CLTS=1", "OK", 1000);               /* время от сети (*PSUTTZ), как в исходнике;
                                                        * модули без этой команды ответят ERROR */
        for (i = 0; i < NET_WAIT_TRIES && !op_expired(); i++) {
            if (gsm_registered()) {
                dbg_stage(4);
                at_cmd("AT+CMGF=1", "OK", 2000);
                set_charset_gsm();
                at_cmd("AT+CNMI=2,1,0,0,0", "OK", 2000);   /* +CMTI о новой SMS будит контроллер */
                if (!time_valid) gsm_clock_time();     /* после рестарта — время от сети */
                process_inbox();
                if (MODULE_SLEEP) at_cmd("AT+ENPWRSAVE=1", "OK", 1000);   /* теперь можно спать */
                dbg_mask |= 8;
                ok = 1;
                break;
            }
            _delay_ms(2000);
        }
    }

    op_end();
    if (alive) uart_idle(); else uart_off();
    ri_enable(ok);
    gsm_fail = 0;
    dbg_leds(0);
    return ok;
}

static uint8_t gsm_send_sms(const char *num, const char *text) {
    if (!num[0]) return 0;
    snprintf(line, sizeof line, "AT+CMGS=\"%s\"", num);   /* line свободна до ответа */
    send_cmd(line);
    if (!wait_char('>', 5000)) { uart_putc(27); return 0; }
    _delay_ms(50);
    uart_puts(text);
    uart_putc(26);
    if (!wait_response("+CMGS", 30000)) return 0;
    wait_response("OK", 2000);
    dbg_flash(1000);
    return 1;
}

static uint8_t inbox_found;

/* 1 — модуль ответил OK (список прочитан), 0 — ERROR или таймаут */
static uint8_t cmgl_read(const char *cmd) {
    uint8_t k, cont = 0;
    send_cmd(cmd);
    while (read_line(line, sizeof line, 3000)) {
        if (!strncmp(line, "+CMGL:", 6)) {
            cont = 0;
            if (!inbox_found) {
                inbox.idx = (uint8_t)atoi(line + 6);
                if (get_quoted(line, 1, big, sizeof big)) ucs2_field(big, inbox.num, EE_STR_LEN);
                else inbox.num[0] = 0;
                inbox.ts[0] = 0;
                for (k = 2; k <= 3; k++)
                    if (get_quoted(line, k, inbox.ts, sizeof inbox.ts) && strchr(inbox.ts, '/')) break;
                if (!strchr(inbox.ts, '/')) inbox.ts[0] = 0;
                read_sms_text(inbox.text, sizeof inbox.text, 2000);   /* следующая строка — текст */
                inbox_found = 1;
                cont = 1;
            } else read_line(big, sizeof big, 2000);                  /* пропустить текст */
        } else if (!strcmp(line, "OK")) return 1;
        else if (strstr(line, "ERROR")) return 0;
        else if (cont) {                         /* следующие строки той же SMS (не UCS2) */
            uint8_t n = strlen(inbox.text);
            if ((uint16_t)n + 2 < sizeof inbox.text) {
                inbox.text[n++] = ';';
                ucs2_field(line, inbox.text + n, sizeof inbox.text - n);
            }
        }
    }
    return 0;
}

/* первая SMS с SIM в inbox; 1 — есть */
static uint8_t gsm_read_inbox(void) {
    uint8_t ucs = 0;
    inbox_found = 0;
    if (READ_UCS2) ucs = at_cmd("AT+CSCS=\"UCS2\"", "OK", 2000);
    if (ucs) {
        /* в UCS2 одни модули ждут "ALL" как есть, другие — тоже в UCS2 */
        if (!cmgl_read("AT+CMGL=\"ALL\"") && !inbox_found)
            cmgl_read("AT+CMGL=\"0041004C004C\"");
        set_charset_gsm();                                            /* отправка — латиницей */
    } else cmgl_read("AT+CMGL=\"ALL\"");
    return inbox_found;
}

static void gsm_delete_sms(uint8_t idx) {
    snprintf(line, sizeof line, "AT+CMGD=%u", idx);
    at_cmd(line, "OK", 5000);
}

/* SMS админу после старта: причина перезагрузки и время.
 * Если время неизвестно — у этой SMS запрашивается отчёт о доставке,
 * в нём сеть указывает время доставки. Обычные настройки SMS потом
 * возвращаются, поэтому на остальную работу это не влияет. */
static void restart_sms(uint8_t rc) {
    const char *why = (rc & (1 << PORF))  ? "PWR" :    /* включили питание          */
                      (rc & (1 << BORF))  ? "BOR" :    /* просадка питания          */
                      (rc & (1 << EXTRF)) ? "RST" :    /* кнопка ресет               */
                      (rc & (1 << WDRF))  ? "WDT" : "?";
    char q[24], ts[24];
    uint8_t k, n, cnmi = 0, report = 0;

    if (!admin[0]) return;
    op_begin(120);
    PCICR &= ~(1 << PCIE2);
    uart_on();

    if (RESTART_TIME_REPORT && !time_valid) {
        cnmi = at_cmd("AT+CNMI=2,1,0,1,0", "OK", 2000);          /* отчёты сразу в UART (+CDS) */
        if (cnmi) report = at_cmd("AT+CSMP=49,167,0,0", "OK", 2000);   /* 49 = с отчётом */
    }

    if (time_valid) snprintf(big, sizeof big, "Restart (%s). Time %02u:%02u", why, hh, mm);
    else if (report) snprintf(big, sizeof big, "Restart (%s)", why);
    else snprintf(big, sizeof big, "Restart (%s). Send any SMS to set time", why);

    if (gsm_send_sms(admin, big) && report) {
        for (k = 0; k < 60 && !op_expired(); k++) {   /* ждём отчёт до ~60 с */
            if (!read_line(line, sizeof line, 1000) || !strstr(line, "+CDS:")) continue;
            ts[0] = 0;
            for (n = 0; n < 4; n++)                    /* последняя метка — время доставки */
                if (get_quoted(line, n, q, sizeof q) && strchr(q, '/')) strcpy(ts, q);
            if (ts[0]) sms_time_update(ts, 'D');
            break;
        }
    }

    op_end();                                          /* возврат настроек — вне лимита */
    if (report) at_cmd("AT+CSMP=17,167,0,0", "OK", 2000);
    if (cnmi) at_cmd("AT+CNMI=2,1,0,0,0", "OK", 2000);
    uart_idle();
    ri_enable(1);
}

/* проверка SIM по RI или раз в минуту */
static void gsm_check(void) {
    uint8_t k, alive = 0;

    op_begin(CHECK_MAX_S);
    PCICR &= ~(1 << PCIE2);                            /* пока говорим с модулем — не будить */
    dbg_mask = 0;
    dbg_stage(1);
    uart_on();
    for (k = 0; k < 5 && !alive; k++) alive = at_cmd("AT", "OK", 700);

    if (alive) {
        gsm_fail = 0;
        dbg_stage(2 | 4);
        at_cmd("ATH", "OK", 1000);                     /* сбросить входящий звонок */
        if (!net_seen) gsm_clock_time();              /* пока от сети времени не было */
        process_inbox();
        dbg_mask |= 8;
    } else if (!MODULE_SLEEP && ++gsm_fail >= 3) {    /* спящий модуль молчит — это норма */
        gsm_ok = 0;                                    /* 3 раза подряд молчит — перезапуск */
        ri_enable(0);
    }

    op_end();
    uart_idle();
    if (gsm_ok) ri_enable(1);
    dbg_leds(0);
}

/* ========================= БАЛАНС ========================= */

static void ucs2_to_ascii(char *s) {
    char *r = s, *w = s;
    while (r[0] && r[1] && r[2] && r[3]) {
        uint16_t code = ((uint16_t)hexval(r[0]) << 12) | ((uint16_t)hexval(r[1]) << 8) |
                        (hexval(r[2]) << 4) | hexval(r[3]);
        *w++ = (code >= 32 && code < 127) ? (char)code : ' ';
        r += 4;
    }
    *w = 0;
}

static void extract_amount(const char *s, char *out, uint8_t max) {
    uint8_t n = 0, dot = 0;
    const char *p = s;
    while (*p && (*p < '0' || *p > '9')) p++;
    if (!*p) { strcpy(out, "N/A"); return; }
    if (p > s && p[-1] == '-') out[n++] = '-';
    while (*p && n < max - 1) {
        if (*p >= '0' && *p <= '9') out[n++] = *p;
        else if (!dot && (*p == '.' || *p == ',') && p[1] >= '0' && p[1] <= '9') { out[n++] = '.'; dot = 1; }
        else break;
        p++;
    }
    out[n] = 0;
}

/* ждёт ответ +CUSD; 1 — пришёл (в big), 0 — нет; в line остаётся последняя строка модуля */
static uint8_t wait_cusd(uint8_t sec) {
    uint8_t k;
    line[0] = 0;
    for (k = 0; k < sec; k++) {
        if (!read_line(big, sizeof big, 1000)) continue;
        if (strstr(big, "+CUSD:")) return 1;
        strncpy(line, big, 40);                       /* для диагностики */
        line[40] = 0;
        if (strstr(big, "ERROR")) return 0;
    }
    return 0;
}

static void do_balance(void) {
    char res[16];
    char *p, *q;
    uint8_t n, got;

    if (!ussd[0]) {
        gsm_send_sms(admin, "No USSD code. Send: balance *100#");
        return;
    }

    snprintf(line, sizeof line, "AT+CUSD=1,\"%s\",15", ussd);
    send_cmd(line);
    got = wait_cusd(30);
    if (!got && strstr(line, "ERROR")) {              /* вариант команды без кодировки */
        snprintf(line, sizeof line, "AT+CUSD=1,\"%s\"", ussd);
        send_cmd(line);
        got = wait_cusd(30);
    }
    if (!got) {                                       /* что ответил модуль — для диагностики */
        snprintf(big, sizeof big, "Balance failed: %s", line[0] ? line : "no answer");
        gsm_send_sms(admin, big);
        return;
    }
    p = strchr(big, '"');
    if (!p) {                                         /* например "+CUSD: 4" — оператор отказал */
        for (n = 0; big[n]; n++) big[n] = gsm_safe(big[n]);
        gsm_send_sms(admin, big);
        return;
    }
    p++;
    q = strchr(p, '"');
    if (q) *q = 0;
    /* длинный ответ в UCS2 мог обрезаться — берём целое число символов */
    for (n = 0; p[n] && hexval(p[n]) != 0xFF; n++);
    if (!p[n] && n >= 8) { p[n & ~3] = 0; ucs2_to_ascii(p); }
    extract_amount(p, res, sizeof res);
    snprintf(big, sizeof big, "Balance: %s rub.", res);
    gsm_send_sms(admin, big);
}

/* ========================= СТАТУС ========================= */

static uint8_t st_len;

static void st_add(const char *fmt, ...) {
    va_list ap;
    int r;
    if (st_len >= sizeof(big) - 1) return;
    va_start(ap, fmt);
    r = vsnprintf(big + st_len, sizeof(big) - st_len, fmt, ap);
    va_end(ap);
    if (r > 0) {
        st_len += (uint8_t)r;
        if (st_len > sizeof(big) - 1) st_len = sizeof(big) - 1;
    }
}

/* R1:0 A:0 L00:00 D0.00 (x5) / Sv:180d / F:1010 / S:100 100 / T:24 H:58 / B:13.5V(crg) */
static void build_status(void) {
    uint8_t i, t, h, v;
    char a[4], bi[4];
    Relay r;

    st_len = 0;
    big[0] = 0;
    for (i = 0; i < 5; i++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { memcpy(&r, (const void *)&relays[i], sizeof r); }
        if (r.auto_en) snprintf(a, sizeof a, "%u", r.auto_h); else strcpy(a, "0");
        if (r.last_h > 23) { r.last_h = 0; r.last_m = 0; }
        st_add("R%u:%u A:%s L%02u:%02u D%u.%02u\n", i + 1, r.on, a,
               r.last_h, r.last_m, r.last_dur / 60, r.last_dur % 60);
    }
    st_add("Sv:%ud\n", servo_angle);
    st_add("F:%u%u%u%u\n", float_stable(0), float_stable(1), float_stable(2), float_stable(3));
    st_add("S:%u %u\n", soil_pct(0), soil_pct(1));
    if (read_dht11(&t, &h)) st_add("T:%u H:%u\n", t, h); else st_add("T:Err\n");

    v = measure_batt_x10();
    if (v > BATT_CHARGE_X10) strcpy(bi, "crg");
    else {
        uint8_t pct = 0;
        if (v >= 127) pct = 100; else if (v >= 124) pct = 75;
        else if (v >= 120) pct = 50; else if (v >= 118) pct = 25;
        snprintf(bi, sizeof bi, "%u", pct);
    }
    st_add("B:%u.%uV(%s)", v / 10, v % 10, bi);
}

/* ========================= КОМАНДЫ ========================= */

static void reply(const char *s) { gsm_send_sms(reply_to, s); }

static uint8_t is_num(const char *p) { return *p >= '0' && *p <= '9'; }

/* "20", "20.5", "20,25" минут -> секунды; 0 — ошибка */
static uint16_t parse_minutes(const char *s) {
    uint32_t sec = 0;
    if (!is_num(s)) return 0;
    while (is_num(s)) {
        sec = sec * 10 + (*s++ - '0');
        if (sec > 1092) return 0;               /* максимум ~18 часов */
    }
    sec *= 60;
    if ((*s == '.' || *s == ',') && is_num(s + 1)) {
        s++;
        sec += (*s++ - '0') * 6;                /* десятые доли минуты */
        if (is_num(s)) sec += ((*s++ - '0') * 6) / 10;
    }
    if (*s) return 0;                           /* лишние символы — ошибка */
    return sec > 65535UL ? 65535U : (uint16_t)sec;
}

/* 1 — команда правильная и выполнена; 0 — переслать админу.
 * adm = 0: разрешены только status, N-on-M, N-off, 6-angle-A. */
static uint8_t execute_command(char *c, uint8_t adm) {
    uint8_t i;

    if (!strcmp(c, "status")) { build_status(); reply(big); return 1; }

    if (c[0] >= '1' && c[0] <= '5' && c[1] == '-') {
        const char *a = c + 2;
        i = c[0] - '1';

        if (!strcmp(a, "off")) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { relay_stop(i); }
            return 1;
        }
        if (!strncmp(a, "on-", 3)) {
            uint16_t sec = parse_minutes(a + 3);
            if (!sec) return 0;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { relay_start(i, sec); }   /* блокировка — видно в status */
            return 1;
        }
        if (!adm) return 0;

        if (!strcmp(a, "auto-off")) {
            relays[i].auto_en = 0;
            save_schedule(i);
            return 1;
        }
        if (!strcmp(a, "auto")) {
            if (relays[i].auto_h > 23 || !relays[i].auto_dur) return 0;   /* нет расписания */
            relays[i].auto_en = 1;
            save_schedule(i);
            return 1;
        }
        if (!strncmp(a, "auto-", 5)) {
            const char *p = a + 5;
            int h = atoi(p), m = 0;
            uint16_t sec = 0;
            if (!is_num(p)) return 0;
            while (is_num(p)) p++;
            if (*p == ':') {
                if (!is_num(++p)) return 0;
                m = atoi(p);
                while (is_num(p)) p++;
            }
            if (*p == '-') sec = parse_minutes(p + 1);
            if (h > 23 || m > 59 || !sec) return 0;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                relays[i].auto_h = h;
                relays[i].auto_m = m;
                relays[i].auto_dur = sec;
                relays[i].auto_en = 1;
            }
            save_schedule(i);
            return 1;
        }
        return 0;
    }

    if (!strncmp(c, "6-angle-", 8)) {
        int a = atoi(c + 8);
        if (!is_num(c + 8) || a > 180) return 0;
        servo_pending = a;                       /* повернём после работы с модулем */
        return 1;
    }

    if (!adm) return 0;

    if (!strncmp(c, "balance", 7) && (c[7] == 0 || c[7] == ' ')) {
        if (c[7] == ' ' && c[8]) {
            strncpy(ussd, c + 8, EE_STR_LEN - 1);
            ussd[EE_STR_LEN - 1] = 0;
            ee_save_str(EE_USSD_ADDR, ussd);
        }
        do_balance();
        return 1;
    }

    /* s1-dry, s2-wet, s1-dry-600 ... */
    if (c[0] == 's' && (c[1] == '1' || c[1] == '2') && c[2] == '-') {
        uint8_t n = c[1] - '1', k;
        const char *p;
        uint16_t v, other;
        if (!strncmp(c + 3, "dry", 3)) k = 0;
        else if (!strncmp(c + 3, "wet", 3)) k = 1;
        else return 0;
        p = c + 6;
        if (*p == 0) v = adc_read(n == 0 ? SOIL1_CH : SOIL2_CH);   /* текущее показание */
        else if (*p == '-' && is_num(p + 1)) v = atoi(p + 1);
        else return 0;
        other = soil_cal[n][!k];
        if (v > 1023 || (v > other ? v - other : other - v) < 30) return 0;   /* точки слишком близко */
        soil_cal[n][k] = v;
        eeprom_update_word((uint16_t *)(EE_CAL_ADDR + n * 4 + k * 2), v);
        return 1;
    }

    /* batt-12.14: реальное напряжение АКБ по мультиметру (до 2 знаков) */
    if (!strncmp(c, "batt-", 5) && is_num(c + 5)) {
        const char *p = c + 5;
        uint16_t v100 = 0, sum;
        uint32_t k;
        while (is_num(p) && v100 < 1000) v100 = v100 * 10 + (*p++ - '0');
        v100 *= 100;
        if ((*p == '.' || *p == ',') && is_num(p + 1)) {
            v100 += (p[1] - '0') * 10;
            p += 2;
            if (is_num(p)) v100 += *p++ - '0';
        }
        if (*p || v100 < 500 || v100 > 2000) return 0;
        sum = batt_raw_x8();
        if (sum < 400) return 0;                      /* делитель не подключён */
        k = ((uint32_t)v100 * 8000UL + sum / 2) / sum;
        if (k < 800 || k > 3300) return 0;            /* плохой контакт в делителе */
        batt_k = (uint16_t)k;
        eeprom_update_word((uint16_t *)(EE_CAL_ADDR + 8), batt_k);
        batt_x10 = measure_batt_x10();
        return 1;
    }

    if (!strcmp(c, "adc")) {
        uint16_t v[6];
        for (i = 0; i < 6; i++) v[i] = adc_read(i);
        snprintf(big, sizeof big, "A0:%u A1:%u A2:%u A3:%u A4:%u A5:%u RI:%u F:%u%u%u%u "
                 "Tm:%02u:%02u first:%c net:%u clts:%u",
                 v[0], v[1], v[2], v[3], v[4], v[5], (PIND >> GSM_RI_PIN) & 1,
                 float_raw(0), float_raw(1), float_raw(2), float_raw(3),   /* поплавки без усреднения */
                 hh, mm, time_src, net_seen, clts_ok);
        reply(big);
        return 1;
    }

    return 0;
}

static void normalize(char *s) {
    char *r = s, *w = s;
    while (*r == ' ' || *r == '"') r++;
    while (*r) {
        char c = *r++;
        if (c >= 'A' && c <= 'Z') c += 32;
        *w++ = c;
    }
    *w = 0;
    while (w > s && (w[-1] == ' ' || w[-1] == '"' || w[-1] == '\r' || w[-1] == '\n')) *--w = 0;
}

/* сравнение по последним 10 цифрам: +7999... == 8999... */
static uint8_t same_number(const char *a, const char *b) {
    uint8_t la = strlen(a), lb = strlen(b);
    if (la < 10 || lb < 10) return strcmp(a, b) == 0;
    return strcmp(a + la - 10, b + lb - 10) == 0;
}

/* есть ли хоть одна буква или цифра (служебные и пустые SMS не пересылаем) */
static uint8_t has_content(const char *s) {
    for (; *s; s++)
        if ((*s >= '0' && *s <= '9') || (*s >= 'a' && *s <= 'z') || (*s >= 'A' && *s <= 'Z')) return 1;
    return 0;
}

/* одна команда (frag — как пришла, для пересылки) */
static void handle_one(const char *from, const char *frag) {
    char text[48];
    uint8_t adm;

    strncpy(text, frag, sizeof text - 1);
    text[sizeof text - 1] = 0;
    normalize(text);
    if (!text[0]) return;
    reply_to = from;

    if (!admin[0]) {                                    /* админ ещё не назначен */
        if (!strcmp(text, "admin")) {
            strncpy(admin, from, EE_STR_LEN - 1);
            admin[EE_STR_LEN - 1] = 0;
            ee_save_str(EE_ADMIN_ADDR, admin);
        } else execute_command(text, 0);               /* пересылать пока некому */
        return;
    }

    adm = same_number(from, admin);
    if (adm && !strcmp(text, "admin")) return;
    if (adm && !strncmp(text, "admin ", 6) && strlen(text + 6) >= 10) {
        strncpy(admin, text + 6, EE_STR_LEN - 1);      /* передать права */
        admin[EE_STR_LEN - 1] = 0;
        ee_save_str(EE_ADMIN_ADDR, admin);
        return;
    }

    if (!execute_command(text, adm) && FORWARD_BAD_SMS && has_content(frag)) {
        snprintf(big, sizeof big, "From %s: %s", from, frag);
        reply_to = admin;
        reply(big);
    }
}

/* SMS может содержать несколько команд через ';' или с новой строки */
static void handle_sms(const char *from, const char *orig) {
    char frag[48];
    const char *p = orig, *e;
    uint8_t len;

    if (!from[0]) return;
    while (*p) {
        for (e = p; *e && *e != ';'; e++);
        len = e - p;
        if (len > sizeof frag - 1) len = sizeof frag - 1;
        memcpy(frag, p, len);
        frag[len] = 0;
        handle_one(from, frag);
        p = *e ? e + 1 : e;
    }
}

/* читать SIM по одной SMS, пока не опустеет */
static void process_inbox(void) {
    uint8_t r;
    for (r = 0; r < 20 && !op_expired(); r++) {
        if (!gsm_read_inbox()) break;
        dbg_flash(200);
        gsm_delete_sms(inbox.idx);           /* сначала удаляем, чтобы не выполнить дважды */
        if (inbox.ts[0]) sms_time_update(inbox.ts, 'S');
        handle_sms(inbox.num, inbox.text);
    }
}

/* ========================= ИНИЦИАЛИЗАЦИЯ, СОН ========================= */

static void io_init(void) {
    /* реле: сначала уровень "выключено", потом выход */
    PORTD |= (1 << REL1_PIN);
    DDRD  |= (1 << REL1_PIN);
    PORTB |= (1 << REL2_PIN) | (1 << REL3_PIN) | (1 << REL4_PIN);
    DDRB  |= (1 << REL2_PIN) | (1 << REL3_PIN) | (1 << REL4_PIN);
    if (REL5_ACTIVE_HIGH) PORTC &= ~(1 << REL5_PIN); else PORTC |= (1 << REL5_PIN);
    DDRC  |= (1 << REL5_PIN);

    PORTD &= ~(1 << SERVO_PIN);
    DDRD  |=  (1 << SERVO_PIN);

    /* поплавки: вход с подтяжкой */
    DDRD &= ~(1 << FLOAT1_PIN);                       PORTD |= (1 << FLOAT1_PIN);
    DDRB &= ~(1 << FLOAT2_PIN);                       PORTB |= (1 << FLOAT2_PIN);
    DDRC &= ~((1 << FLOAT3_PIN) | (1 << FLOAT4_PIN)); PORTC |= (1 << FLOAT3_PIN) | (1 << FLOAT4_PIN);

    DDRD &= ~(1 << DHT_PIN); PORTD |= (1 << DHT_PIN);

    /* GSM: RI и K в Hi-Z */
    DDRD  &= ~((1 << GSM_RI_PIN) | (1 << GSM_PWR_PIN));
    PORTD &= ~((1 << GSM_RI_PIN) | (1 << GSM_PWR_PIN));

    /* свободные PB4..PB7 — вход с подтяжкой */
    DDRB  &= ~0xF0;
    PORTB |=  0xF0;

    DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC5D);
    ACSR  = (1 << ACD);
    ADCSRA = 0;
    PRR = (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM1) | (1 << PRTIM0) |
          (1 << PRSPI) | (1 << PRUSART0);
    uart_off();
}

static void wdt_setup(void) {
    cli();
    wdt_reset();
    WDTCSR = (1 << WDCE) | (1 << WDE);
    WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);  /* прерывание ~8 с */
}

static void go_sleep(void) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    if (!ri_flag) {
        sleep_enable();
        sleep_bod_disable();
        sei();
        sleep_cpu();
        sleep_disable();
    }
    sei();
}

int main(void) {
    uint8_t i, poll_min = 0, retry_min = 0, restart_sent = 0;
    uint8_t rc = MCUSR;                  /* причина перезагрузки */

    MCUSR = 0;
    wdt_disable();

    io_init();
    settings_load();
    batt_x10 = measure_batt_x10();
    for (i = 0; i < 4; i++) float_hist[i] = float_raw(i) ? 0x07 : 0x00;

    dbg_boot();
    servo_move(servo_angle);         /* до включения GSM, чтобы не было просадки */

    wdt_setup();
    sei();

    gsm_ok = gsm_start();
    if (gsm_ok && RESTART_SMS) { restart_sms(rc); restart_sent = 1; }

    for (;;) {
        uint8_t poll = 0;

        if (minute_flag) {
            minute_flag = 0;
            batt_x10 = measure_batt_x10();
            dbg_result();
            if (gsm_ok) {
                if (++poll_min >= (MODULE_SLEEP ? SLEEP_POLL_MIN : SMS_POLL_MIN)) { poll_min = 0; poll = 1; }
            } else if (++retry_min >= GSM_RETRY_MIN) {
                retry_min = 0;
                gsm_ok = gsm_start();
                if (gsm_ok && RESTART_SMS && !restart_sent) { restart_sms(rc); restart_sent = 1; }
            }
        }

        if (gsm_ok && (ri_flag || poll)) {
            ri_flag = 0;
            poll_min = 0;
            gsm_check();
        }
        ri_flag = 0;

        if (servo_pending <= 180) {
            servo_move(servo_pending);
            servo_pending = 0xFF;
        }

        go_sleep();
    }
}
