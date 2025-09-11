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
        // If we found a button . . . //！！FIX -> 同时按同一列的两个按键
        if (keypad & button) {
            for (i=0; i<NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ; //查找对应的key code
            }
            if (i==NUMKEYS) (i = -1) ; //如果查找第12个仍没有对应，则表示没有找到对应的key code，返回-1
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
                    printf("Key %d pressed\r\n", i) ;       //打印按键编号 -> 后续增加beep
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

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

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
