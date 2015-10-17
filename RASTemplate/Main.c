#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/pwm.h>
#include <math.h>

// Blink the LED to show we're on
// tBoolean blink_on = true;
tPWM * green;
tPWM * red;
tPWM * blue;

float light_counter = 0;
float light_offset = 1;
float light_cycle_time = 1;
float light_refresh_step = 0.01;
float light_step = 1;

void init(){
    light_offset = 3.14159*2/3;
    light_step = 2*3.14159*light_refresh_step/light_cycle_time;
}

void change_pwm(){
    light_counter += light_step;
    if(light_counter >=  2*3.14159){
        light_counter = 0;
    }
    SetPWM(green, sin(light_counter)*0.5+0.5, 0);
    SetPWM(red, sin(light_counter+light_offset)*0.5+0.5, 0);
    SetPWM(blue, sin(light_counter-light_offset)*0.5+0.5, 0);
}

void blink(void) {
    //SetPin(PIN_B0, blink_on);
    //blink_on = !blink_on;
    //SetPin(PIN_F1, blink_on);
    //SetPin(PIN_F2, blink_on);
    //SetPin(PIN_F3, blink_on);
}


// The 'main' function is the entry point of the program
int main(void) {
    init();

    green = InitializePWM(PIN_F3, 500);
    red = InitializePWM(PIN_F1, 500);
    blue = InitializePWM(PIN_F2, 500);

    CallEvery(change_pwm, 0, light_refresh_step);

    while (1) {
        Printf("Hello World!\n");
    }
}
