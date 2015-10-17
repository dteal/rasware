#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/pwm.h>
#include <math.h>

// Blink the LED to show we're on
tBoolean blink_on = true;
tPWM * green;
tPWM * red;
tPWM * blue;
float i = 0;
float pi = 3.14159;


void change_pwm(){
    i+=0.01;
    if(i > 1000){
        i = 0;
    }
    SetPWM(green, sin(i)*0.5+0.5, 0);
    SetPWM(red, sin(i+pi)*0.5+0.5, 0);
    SetPWM(blue, sin(i-pi)*0.5+0.5, 0);
}

void blink(void) {
    SetPin(PIN_B0, blink_on);
    blink_on = !blink_on;
    //SetPin(PIN_F1, blink_on);
    //SetPin(PIN_F2, blink_on);
    //SetPin(PIN_F3, blink_on);
}


// The 'main' function is the entry point of the program
int main(void) {
    pi = pi * 2 / 3;

    green = InitializePWM(PIN_F3, 100);
    red = InitializePWM(PIN_F1, 100);
    blue = InitializePWM(PIN_F2, 100);

    CallEvery(change_pwm, 0, 0.01);

    while (1) {
        Printf("Hello World!\n");
    }
}
