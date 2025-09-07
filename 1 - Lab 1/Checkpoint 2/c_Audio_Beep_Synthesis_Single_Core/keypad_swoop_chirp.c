// Keypad "1" -> swoop ; "2" -> chirp
// DDS + SPI DAC at 50 kHz; amplitude attack/decay ramps; debounced keypad
// Based on Hunter Adams' beep_beep.c (DDS+DAC+ISR) and keypad.c (4x3 scan + VGA)

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

#include "vga16_graphics_v2.h"
#include "pt_cornell_rp2040_v1_4.h"

//========================= Fixed-point helpers =========================//
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0f))
#define fix2int15(a)   ((int)((a)>>15))
#define int2fix15(a)   ((fix15)((a)<<15))
#define divfix(a,b)    ((fix15)((((signed long long)(a))<<15)/(b)))

//========================= DDS / Audio constants =======================//
#define two32 4294967296.0  // 2^32
#define Fs    50000         // 50 kHz audio
#define DELAY 20            // 1/Fs (us)

// DAC/SPI pins
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define SPI_PORT spi0

// DAC config (MCP49xx family) - channel B, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// Scope pin for ISR timing
#define ISR_GPIO 2

//========================= Keypad / UI pins ===========================//
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

// >>> Modify these two indices if your keypad wiring maps differently <<<
#define KEY_INDEX_FOR_1 1   // if pressing '1' prints index 4, change to 4
#define KEY_INDEX_FOR_2 2   // if pressing '2' prints index 5 (or 2/??), change here

static const unsigned int keycodes[NUMKEYS] = {
  0x28, 0x11, 0x21, 0x41, 0x12, 0x22, 0x42, 0x14, 0x24, 0x44, 0x18, 0x48
};
static const unsigned int scancodes[KEYROWS] = {0x01, 0x02, 0x04, 0x08};
static const unsigned int button_mask = 0x70;

//========================= Sine table ================================//
#define SINE_TABLE_SIZE 256
static fix15 sin_table[SINE_TABLE_SIZE];

//========================= Sound engine state ========================//
enum SOUND_REQ { REQ_NONE=0, REQ_SWOOP=1, REQ_CHIRP=2 };

static volatile uint8_t  sound_request = REQ_NONE;
static volatile bool     sound_active  = false;

static volatile unsigned int phase_accum = 0;
static volatile unsigned int phase_incr  = 0;   // current phase increment (Hz -> incr)
static volatile int          DAC_output  = 0;

// frequency sweep
static volatile int   total_samples    = 0;
static volatile int   sample_count     = 0;
static volatile int   attack_samples   = 0;
static volatile int   decay_samples    = 0;
static volatile int   sustain_samples  = 0;     // optional (not required)
static volatile int   phase_step       = 0;     // per-sample delta of phase_incr (signed)

// amplitude envelope
static fix15 max_amplitude     = int2fix15(1);
static volatile fix15 current_amplitude = 0;
static volatile fix15 attack_inc = 0;
static volatile fix15 decay_inc  = 0;

//========================= Utility: Hz --> phase incr =================//
static inline unsigned int hz_to_incr(float hz) {
    return (unsigned int)((hz * two32) / (float)Fs);
}

//========================= Sound presets =============================//
// Tuned for clean, characteristic sounds at 50 kHz Fs.
// You can tweak the numbers to taste.

// --- “Swoop”: downward glide ---
#define SWOOP_START_HZ   1200.0f
#define SWOOP_END_HZ      300.0f
#define SWOOP_MS          220      // total duration
#define SWOOP_ATTACK_MS     5
#define SWOOP_DECAY_MS     12

// --- “Chirp”: short upward sweep ---
#define CHIRP_START_HZ    600.0f
#define CHIRP_END_HZ     2200.0f
#define CHIRP_MS           80
#define CHIRP_ATTACK_MS     2
#define CHIRP_DECAY_MS      8

//========================= Configure one-shot sound ==================//
static void start_sound(enum SOUND_REQ kind) {
    // Set per-sound timing
    int dur_ms, att_ms, dec_ms;
    unsigned int incr_start, incr_end;

    if (kind == REQ_SWOOP) {
        dur_ms    = SWOOP_MS;
        att_ms    = SWOOP_ATTACK_MS;
        dec_ms    = SWOOP_DECAY_MS;
        incr_start = hz_to_incr(SWOOP_START_HZ);
        incr_end   = hz_to_incr(SWOOP_END_HZ);
    } else { // REQ_CHIRP
        dur_ms    = CHIRP_MS;
        att_ms    = CHIRP_ATTACK_MS;
        dec_ms    = CHIRP_DECAY_MS;
        incr_start = hz_to_incr(CHIRP_START_HZ);
        incr_end   = hz_to_incr(CHIRP_END_HZ);
    }

    total_samples   = (dur_ms * Fs) / 1000;
    if (total_samples < 10) total_samples = 10;

    attack_samples  = (att_ms * Fs) / 1000;
    if (attack_samples < 1) attack_samples = 1;

    decay_samples   = (dec_ms * Fs) / 1000;
    if (decay_samples < 1) decay_samples = 1;

    if (attack_samples + decay_samples > total_samples)
        decay_samples = total_samples - attack_samples;

    sustain_samples = total_samples - attack_samples - decay_samples;

    // Envelope increments (fix15)
    attack_inc = divfix(max_amplitude, int2fix15(attack_samples));
    decay_inc  = divfix(max_amplitude, int2fix15(decay_samples));
    current_amplitude = 0;

    // Frequency glide: linear in phase_incr (smooth in frequency)
    phase_incr  = incr_start;
    int delta   = (int)incr_end - (int)incr_start;
    phase_step  = delta / total_samples;

    // Phase reset for clean start (no edge click)
    phase_accum = 0;

    // Reset counters and go
    sample_count  = 0;
    sound_active  = true;
}

//========================= Timer ISR (Alarm0) ========================//
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

static void alarm_irq(void) {
    // ISR entry flag
    gpio_put(ISR_GPIO, 1);

    // Clear + re-arm Alarm0 for next tick
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Latch a new request if idle
    if (!sound_active && sound_request != REQ_NONE) {
        enum SOUND_REQ req = (enum SOUND_REQ)sound_request;
        sound_request = REQ_NONE;
        start_sound(req);
    }

    if (sound_active) {
        // ---- DDS: phase & table ----
        phase_accum += phase_incr;
        fix15 s_fix = multfix15(current_amplitude,
                                sin_table[phase_accum >> 24]);
        DAC_output = (fix2int15(s_fix)) + 2048;

        // ---- amplitude envelope ----
        if (sample_count < attack_samples) {
            current_amplitude += attack_inc;
            if (current_amplitude > max_amplitude) current_amplitude = max_amplitude;
        } else if (sample_count >= (attack_samples + sustain_samples)) {
            // decay stage
            if (current_amplitude > 0) current_amplitude -= decay_inc;
            if (current_amplitude < 0) current_amplitude = 0;
        }

        // ---- frequency sweep ----
        phase_incr += phase_step;

        // ---- write to DAC ----
        uint16_t dac_word = (uint16_t)(DAC_config_chan_B | (DAC_output & 0x0FFF));
        spi_write16_blocking(SPI_PORT, &dac_word, 1);

        // ---- advance / stop ----
        sample_count++;
        if (sample_count >= total_samples) {
            // park DAC at midscale to avoid DC
            uint16_t mid = (uint16_t)(DAC_config_chan_B | 2048);
            spi_write16_blocking(SPI_PORT, &mid, 1);

            sound_active = false;
            sample_count = 0;
            current_amplitude = 0;
        }
    }

    // ISR exit flag
    gpio_put(ISR_GPIO, 0);
}

//========================= Keypad + UI thread ========================//
static PT_THREAD (protothread_core_0(struct pt *pt)) {
    PT_BEGIN(pt);

    static int i, r;
    static uint32_t keypad_bits;

    // Debounce state
    static int      last_raw = -1;
    static int      stable_key = -1, prev_stable = -1;
    static uint64_t last_change_us = 0;
    const  uint32_t debounce_us = 20000; // 20 ms

    char keytext[32];

    while (1) {
        // --- scan keypad: drive one row high, read columns ---
        i = -1;
        for (r = 0; r < KEYROWS; r++) {
            gpio_put_masked((0xF << BASE_KEYPAD_PIN), (scancodes[r] << BASE_KEYPAD_PIN));
            sleep_us(1);
            keypad_bits = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F);
            if (keypad_bits & button_mask) {
                for (int k = 0; k < NUMKEYS; k++) {
                    if (keypad_bits == keycodes[k]) { i = k; break; }
                }
                break; // first hit only
            }
        }
        // rows low
        gpio_put_masked((0xF << BASE_KEYPAD_PIN), 0);

        // --- debouncing ---
        if (i != last_raw) { last_raw = i; last_change_us = time_us_64(); }
        uint64_t now = time_us_64();
        if ((now - last_change_us) >= debounce_us && i != stable_key) {
            stable_key = i;

            if (stable_key != prev_stable) {
                prev_stable = stable_key;

                // UI
                fillRect(250, 20, 176, 30, RED);
                sprintf(keytext, "%d", stable_key);
                setCursor(250, 20); setTextSize(2);
                writeString(keytext);

                // serial print
                printf("\\nkey=%d", stable_key);

                // ---- Trigger sounds on press edge only ----
                if (stable_key >= 0) {
                    if (stable_key == KEY_INDEX_FOR_1) {
                        sound_request = REQ_SWOOP;  // '1' -> swoop
                    } else if (stable_key == KEY_INDEX_FOR_2) {
                        sound_request = REQ_CHIRP;  // '2' -> chirp
                    }
                }
            }
        }

        // Blink LED every ~320 ms
        static uint32_t led_div = 0;
        if ((led_div++ & 0x3F) == 0) gpio_put(LED, !gpio_get(LED));

        PT_YIELD_usec(5000); // ~5 ms loop
    }

    PT_END(pt);
}

//========================= main =====================================//
int main() {
    // Overclock for VGA timing (from keypad demo)
    set_sys_clock_khz(150000, true);

    stdio_init_all();

    // SPI/DAC init
    spi_init(SPI_PORT, 20000000);
    spi_set_format(SPI_PORT, 16, 0, 0, 0);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SPI);

    gpio_init(LDAC);  gpio_set_dir(LDAC, GPIO_OUT);  gpio_put(LDAC, 0);
    gpio_init(ISR_GPIO); gpio_set_dir(ISR_GPIO, GPIO_OUT); gpio_put(ISR_GPIO, 0);
    gpio_init(LED);  gpio_set_dir(LED, GPIO_OUT); gpio_put(LED, 0);

    // Sine table (±2047)
    for (int ii = 0; ii < SINE_TABLE_SIZE; ii++) {
        sin_table[ii] = float2fix15(2047.0f * sinf((float)ii * 6.2831853f / (float)SINE_TABLE_SIZE));
    }

    // VGA init / title bar
    initVGA();
    fillRect(64, 0, 176, 50, BLUE);
    fillRect(250, 0, 176, 50, RED);
    fillRect(435, 0, 176, 50, GREEN);
    setTextColor(WHITE);
    setCursor(65, 0);  setTextSize(1);
    writeString("Raspberry Pi Pico");
    setCursor(65, 10); writeString("Keypad swoop/chirp");
    setCursor(65, 20); writeString("Press 1 -> swoop");
    setCursor(65, 30); writeString("Press 2 -> chirp");
    setCursor(250, 0); setTextSize(2); writeString("Key Pressed:");

    // Keypad GPIO
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN));
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)); // rows as output
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), 0);
    gpio_pull_down(BASE_KEYPAD_PIN + 4);
    gpio_pull_down(BASE_KEYPAD_PIN + 5);
    gpio_pull_down(BASE_KEYPAD_PIN + 6);

    // Timer Alarm0 ISR
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    irq_set_enabled(ALARM_IRQ, true);
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Protothread
    pt_add_thread(protothread_core_0);
    pt_schedule_start;

    while (1) { tight_loop_contents(); }
}
