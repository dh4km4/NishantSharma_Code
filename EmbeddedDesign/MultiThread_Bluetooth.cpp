#include "mbed.h"
#include "rtos.h"
#include "uLCD_4DGL.h"
#include "SDFileSystem.h"
#include "wave_player.h"
#include "rgbLED.h"

SDFileSystem sd(p5, p6, p7, p8, "sd"); //SD card
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
// BusOut myled(LED1,LED2,LED3,LED4);
uLCD_4DGL uLCD(p9,p10,p30); // serial tx, serial rx, reset pin;
Mutex uLCDMutex;
// Serial pc(USBTX, USBRX);
Serial Blue(p28, p27);
Mutex modeMutex;
Mutex BlueMutex;
int mode = 0;

AnalogOut DACout(p18);
wave_player waver(&DACout);

RGBLed myLed(p23, p22, p21);


const LEDColor red(1.0,0.0,0.0);
const LEDColor green(0.0,0.2,0.0);
//brighter green LED is scaled down to same as red and
//blue LED outputs on Sparkfun RGBLED
const LEDColor blue(0.0,0.0,1.0);
const LEDColor yellow(1.0,0.2,0.0);
const LEDColor white(1.0,0.2,1.0);
const LEDColor purple = red+blue;
const LEDColor black(0.0, 0.0, 0.0);

void lcd_time_thread(void const *argument) {
    while (true) {
        time_t sec = time(NULL);
        char str_time[32];
        struct tm *local_UTC;
        local_UTC = localtime(&sec);
        local_UTC->tm_hour = (local_UTC->tm_hour);
        local_UTC->tm_min = (local_UTC->tm_min);
        local_UTC->tm_sec = (local_UTC->tm_sec + 25);
        mktime(local_UTC);
        strftime(str_time, 32, "%I:%M:%S %p\n", local_UTC);
        uLCDMutex.lock();
        uLCD.text_string(str_time, 5, 0, FONT_7X8, WHITE);
        uLCDMutex.unlock();
        Thread::wait(500);
    }
}

void lcd_image_thread(void const *argument) {  
    while(true) {
        modeMutex.lock();
        int lcd_mode = mode;
        modeMutex.unlock();
        switch(lcd_mode) {
            case 1:
                uLCDMutex.lock();
                uLCD.media_init();
                uLCD.set_byte_address(0x0000, 0xBC00);   // Firework 1
                uLCD.display_image(0, 35);
                uLCDMutex.unlock();
                Thread::wait(1000);
                break;
            case 2:
                uLCDMutex.lock();
                uLCD.media_init();
                uLCD.set_byte_address(0x0000, 0x5E00);   // Firework 2
                uLCD.display_image(0, 35);
                uLCDMutex.unlock();
                Thread::wait(1000); 
                break;       
            case 3:
                uLCDMutex.lock();
                uLCD.media_init();
                uLCD.set_byte_address(0x0000, 0x0000);   // Water
                uLCD.display_image(0, 35);
                uLCDMutex.unlock();
                Thread::wait(1000);
                break;
            case 4:
                uLCDMutex.lock();
                uLCD.media_init();
                uLCD.set_byte_address(0x0001, 0x1A00);   // Thunder
                uLCD.display_image(0, 35);
                uLCDMutex.unlock();
                Thread::wait(1000);
                break;
            default:
                uLCDMutex.lock();
                uLCD.filled_rectangle(0, 35 , 128, 128, BLACK);
                uLCDMutex.unlock();
                Thread::wait(500);
                break;
        }
    }
}

void speaker_thread(void const *argument) {
    while(true) {
        FILE *firework_1 = fopen("/sd/Sounds/Firework_1.wav", "r");
        FILE *firework_2 = fopen("/sd/Sounds/Firework_2.wav", "r");
        FILE *water = fopen("/sd/Sounds/Water.wav", "r");
        FILE *thunder = fopen("/sd/Sounds/Thunder.wav", "r");
        
        modeMutex.lock();
        int speaker_mode = mode;
        modeMutex.unlock();
        
        switch(speaker_mode) {
            case 1:
                Thread::wait(1000);
                waver.play(firework_1);
                Thread::wait(2000);        
                break;
            case 2:
                Thread::wait(1000);
                waver.play(firework_2);
                Thread::wait(2000);                    
                break;
            case 3:
                Thread::wait(1000);
                waver.play(water);
                Thread::wait(2000);        
                break;
            case 4:
                Thread::wait(1000);
                waver.play(thunder);
                Thread::wait(2000);
                break;
            default:
                break;
        }        
        modeMutex.lock();
        mode = 0;
        modeMutex.unlock();
        fclose(firework_1);
        fclose(firework_2);
        fclose(water);
        fclose(thunder);
    }
}

void led_thread(void const *argument) {
    while(true) {
        modeMutex.lock();
        int led_mode = mode;
        modeMutex.unlock();
        switch(led_mode) {
            case 1:
                myLed = (red+yellow)*0.2;
                Thread::wait(700);
                myLed = black;
                Thread::wait(700);
                break;
            case 2:
                myLed = yellow;
                Thread::wait(400);
                myLed = (blue+white)*0.2;
                Thread::wait(400);
                myLed = (green+white)*0.2;
                Thread::wait(400);
                myLed = (red+blue)*0.2;
                Thread::wait(400);
                break;
            case 3:
                for (float brightness = 0.0; brightness < 0.2; brightness+=0.001) {
                    myLed = (blue + white)*brightness;
                    Thread::wait(5);
                }
                for (float brightness = 0.2; brightness > 0.0; brightness-=0.001) {
                    myLed = (blue + white)*brightness;
                    Thread::wait(5);
                }
                break;
            case 4:
                myLed = purple*0.2;
                Thread::wait(700);
                myLed = black;
                Thread::wait(300);
                break;
            default:
                myLed = black;
                break;
        }
    }
}

void flushSerialBuffer(void) { 
    char char1 = 0; 
    while (Blue.readable()) { 
        uLCDMutex.lock();    
        char1 = Blue.getc(); 
        uLCDMutex.unlock();
    }
    return; 
}


int main() {
    uLCDMutex.lock();
    uLCD.cls();
    uLCDMutex.unlock();
    Thread::wait(1000);
    Thread t1(lcd_time_thread, NULL, osPriorityNormal, (DEFAULT_STACK_SIZE));
    Thread t2(lcd_image_thread, NULL, osPriorityNormal, (DEFAULT_STACK_SIZE));
    Thread t3(speaker_thread, NULL, osPriorityNormal, (DEFAULT_STACK_SIZE));
    Thread t4(led_thread, NULL, osPriorityNormal, (DEFAULT_STACK_SIZE));
    char bnum = 0;
    char bhit = 0;
    while (true) {
        // led1 = !led1;
        // Thread::wait(500);
        modeMutex.lock();
        int main_mode=mode;
        modeMutex.unlock();
        if (main_mode == 0) {
            led1=0;
            led2=0;
            led3=0;
            led4=0;
            if (Blue.readable()) {
                uLCDMutex.lock();
                if (Blue.getc()=='!') {
                    if (Blue.getc()=='B') { //button data
                        bnum = Blue.getc(); //button number
                        bhit = Blue.getc();
                        uLCDMutex.unlock();
                        if (bhit=='1') {
                        switch(bnum) {
                            case '1':
                                main_mode=1;
                                led1=1;
                                break;
                            case '2':
                                main_mode=2;
                                led2=1;
                                break;
                            case '3':
                                main_mode=3;
                                led3=1;
                                break;
                            case '4':
                                main_mode=4;
                                led4=1;
                                break;
                        }
                        }
                    }
                }
                uLCDMutex.unlock();
            }
            modeMutex.lock();
            mode=main_mode;
            modeMutex.unlock();
        }
        Thread::wait(500);
    }
}
