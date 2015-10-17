#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>

// Blink the LED to show we're on
tBoolean blink_on = true;

void blink(void) {
    SetPin(PIN_B0, blink_on);
    blink_on = !blink_on;
    SetPin(PIN_F1, blink_on);
    SetPin(PIN_F2, blink_on);
    SetPin(PIN_F3, blink_on);
}


// The 'main' function is the entry point of the program
int main(void) {
    // Initialization code can go here
<<<<<<< HEAD
    CallEvery(blink, 0, 0.1);
=======
    CallEvery(blink, 0, 0.5);
    
>>>>>>> 38721293b469304b65d081ec8dd8ed66381b8629
    while (1) {
        // Runtime code can go here
        Printf("Hello World!\n");
    }
}
