/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * Keypad Demo
 * 
 * KEYPAD CONNECTIONS
 *  - GPIO 9   -->  330 ohms  --> Pin 1 (button row 1)
 *  - GPIO 10  -->  330 ohms  --> Pin 2 (button row 2)
 *  - GPIO 11  -->  330 ohms  --> Pin 3 (button row 3)
 *  - GPIO 12  -->  330 ohms  --> Pin 4 (button row 4)
 *  - GPIO 13  -->     Pin 5 (button col 1)
 *  - GPIO 14  -->     Pin 6 (button col 2)
 *  - GPIO 15  -->     Pin 7 (button col 3)
 * 
 * SERIAL CONNECTIONS
 *  - GPIO 0        -->     UART RX (white)
 *  - GPIO 1        -->     UART TX (green)
 *  - RP2040 GND    -->     UART GND
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/spi.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "pt_cornell_rp2040_v1_4.h"

// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

#define LED             25

unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, // 0, 1, 2, 3
                                0x12, 0x22, 0x42, 0x14, // 4, 5, 6, 7
                                0x24, 0x44, 0x18, 0x48, // 8, 9, *, #
                            } ;             
                                          
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;//逐行激活，row1(gpio9) - row4(gpio12)

unsigned int button = 0x70 ; //对应1110000 => 如果0x70 & keypad != 0 则表示有按键按下，当有按键被按下，gpio15，14，13中至少有一个为高电平

typedef enum {//添加状态机
    Not_Pressed = 0,
    Maybe_Pressed,
    Pressed,
    Maybe_Not_Pressed
} key_state;

static key_state key_status = Not_Pressed ;
//添加beep触发器

volatile bool beep_trigger = false ;

//=========================================================Alarm interrupt for generating sound=========================================================
// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             250
#define DECAY_TIME              250
#define SUSTAIN_TIME            10000
#define BEEP_DURATION           10500
#define BEEP_REPEAT_INTERVAL    50000

// State machine variables
volatile unsigned int STATE_0 = 1 ;
volatile unsigned int count_0 = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define SPI_PORT spi0

//GPIO for timing the ISR
#define ISR_GPIO 2
//=============================================================================================================================================


// This timer ISR is called on core 0
static void alarm_irq(void) {

    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    if (STATE_0 == 0) {
        // DDS phase and sine table lookup
        phase_accum_main_0 += phase_incr_main_0  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;

        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_A | (DAC_output_0 & 0xffff))  ; // Change DAC_config_chan_B to DAC_config_chan_A would change the channel

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;

        // State transition?
        if (count_0 == BEEP_DURATION) {
            STATE_0 = 1 ;
            count_0 = 0 ;
            current_amplitude_0 = 0 ;
        }
    }

    // State transition?
    else {
        if (beep_trigger) {
            beep_trigger = false ;
            current_amplitude_0 = 0 ;
            STATE_0 = 0 ;
            count_0 = 0 ;
        }
    }

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0) ;

}
// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    static int i ;
    static uint32_t keypad ;

    while(1) { 
        gpio_put(LED, !gpio_get(LED)) ;//LED闪烁

        // Scan the keypad!
        for (i=0; i<KEYROWS; i++) {//for循环逐行扫描
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN), //mask以选择gpio pin 9-12，左移9位，因为共32位对应gpio0-31
                            (scancodes[i] << BASE_KEYPAD_PIN)) ; //选择激活一行
            // Small delay required
            sleep_us(1) ; 
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ; //右移9位使得gpio9-15对应最低7位，然后与0x7F和运算以屏蔽其他位，取用最低7位
            // Break if button(s) are pressed
            if (keypad & button) break ; //和0x70和运算，若结果不为0则表示有按键按下，跳出for循环
        }
        // If we found a button . . .
        if (keypad & button) {
            for (i=0; i<NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ; //查找对应的key code
            }
            if (i==NUMKEYS) (i = -1) ; //如果查找第12个yes，则表示没有找到对应的key code，返回-1
        }
        else (i=-1) ; //如果没有按键按下，返回-1

        // 状态机更新
        switch (key_status) {
            case Not_Pressed:
                if (i != -1) key_status = Maybe_Pressed;    //如果监测到keycode不为-1，表示有按键按下，进入Maybe_Pressed状态
                else key_status = Not_Pressed ;             //如果没有检测到按键按下，继续保持Not_Pressed状态
                break;
            case Maybe_Pressed:
                if (i == -1) key_status = Not_Pressed;      //如果监测到keycode位-1，便是没有按键按下，返回到Not_Pressed状态
                else 
                {
                    key_status = Pressed;                   //如果监测到keycode不为-1，表示有按键按下，进入Pressed状态
                    //printf("Key %d pressed\r\n", i) ;     //打印按键编号 -> 后续增加beep
                    beep_trigger = true ;                      //触发beep Trigger
                }
                break;
            case Pressed:
                if (i == -1) key_status = Maybe_Not_Pressed;//如果监测到keycode位-1，便是没有按键按下，进入Maybe_Not_Pressed状态
                else key_status = Pressed ;                 //如果监测到keycode不为-1，表示有按键按下，继续保持Pressed状态
                break;
            case Maybe_Not_Pressed:
                if (i != -1) key_status = Pressed;          //如果监测到keycode不为-1，表示有按键按下，返回Pressed状态
                else key_status = Not_Pressed;              //如果监测到keycode位-1，便是没有按键按下，进入Not_Pressed状态
                break;
        }

        PT_YIELD_usec(30000) ; //线程每30ms尝试运行一次，如果cpu核心被占用则等待下一次
    }
    // Indicate thread end
    PT_END(pt) ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ; //150MHz system clock up to 133MHz

    // Initialize stdio
    stdio_init_all();

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO) ;
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;
    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }
    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM) ;
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq) ;
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true) ;
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ; //initialize gpio9-15
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;  //set gpio9-12 to output
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN)) ; //set gpio9-12 to low
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_down((BASE_KEYPAD_PIN + 4)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 5)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 6)) ;

    // Add core 0 threads
    pt_add_thread(protothread_core_0) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;
}
