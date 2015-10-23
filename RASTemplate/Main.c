#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/pwm.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/adc.h>
#include <math.h>

// LIGHT MANAGEMENT =============
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
// ==============================

// IO COMPONENTS ================
tMotor * left_motor;
tMotor * right_motor;
tADC * ir_sensor;
int ir_average_length = 10;
float ir_average_table[10] = {0};
int ir_average_index = 0;
// ==============================

// Initialize light variables
void initialize_led_variables(){
    light_offset = 3.14159*2/3;
    light_step = 2*3.14159*light_refresh_step/light_cycle_time;
}

// Change light brightness
void change_led_pwm(){
    light_counter += light_step;
    if(light_counter >=  2*3.14159){
        light_counter = 0;
    }
    SetPWM(green, sin(light_counter)*0.5+0.5, 0);
    SetPWM(red, sin(light_counter+light_offset)*0.5+0.5, 0);
    SetPWM(blue, sin(light_counter-light_offset)*0.5+0.5, 0);
}

// Adjusts motor speeds based on IR sensor reading
void adjust_motor_power(){

    // Get value from IR sensor and average
    ir_average_table[ir_average_index] = ADCRead(ir_sensor);
    ir_average_index++;
    if(ir_average_index>=ir_average_length+1){
        ir_average_index = 0;
    }
    float sum = 0;
    int i = 0;
    for(i; i < ir_average_length; i++){
        sum += ir_average_table[i];
    }
    float value = sum / ir_average_length;

    // The ideal IR sensor value
    float middle_value = 0.5;

    // How much the motors react to the IR sensor
    float responsiveness = 0.5;

    // How fast the robot drives straight
    float speed = 0.5;

    // Adjust motor speeds
    SetMotor(left_motor, (value-middle_value)*responsiveness+speed);
    SetMotor(right_motor, (value+middle_value)*responsiveness+speed);
}

int main(void) {

    // Initialize IR sensor
    ir_sensor = InitializeADC(PIN_A4);

    // Initialize RGB LEDs
    green = InitializePWM(PIN_F3, 500);
    red = InitializePWM(PIN_F1, 500);
    blue = InitializePWM(PIN_F2, 500);
    initialize_led_variables();

    // Initialize motors
    left_motor = InitializeServoMotor(PIN_A2, false);
    right_motor = InitializeServoMotor(PIN_A3, false);

    // Start LED color-changing
    CallEvery(change_led_pwm, 0, light_refresh_step);

    // Start line following
    ADCBackgroundRead(ir_sensor, adjust_motor_power, NULL);

    // Infinite loop so that program keeps running
    while(1){}
}
