#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/pwm.h>
#include <math.h>

// Blink the LED to show we're on
tBoolean blink_on = true;
tPWM * green;
float i = 0;

void change_pwm(){
    i+=0.01;
    if(i > 1){
        i = 0;
    }
    SetPWM(green, i, 0);
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
    float pi = 3.142f;
    

    green = InitializePWM(PIN_F3, 100);

    CallEvery(change_pwm, 0, 0.01);

    while (1) {
        Printf("Hello World!\n");
    }
}
