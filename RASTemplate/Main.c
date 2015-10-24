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

float coerce(float val, float min, float max){
    if(val < min){
        val=min;
    }
    if(val > max){
        val=max;
    }
    return val;
}

// Adjusts motor speeds based on IR sensor reading
void adjust_motor_power(){

    // Get value from IR sensor and average
    float temp = ADCRead(ir_sensor);
    //Printf("%f\n", temp);
    ir_average_table[ir_average_index] = temp;
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
    value = temp;

    // 12"  .3
    // 9"   .35
    // 6"   .54
    // 5"   .62
    // 4"   .74
    // 3"   .87
    // 2"   .94
    // 1"   (fluctuates)

    // The ideal IR sensor value
    float middle_value = 0.74;
    float signal_width = 0.2;
    if(value < middle_value-signal_width){
        value = middle_value-signal_width;
    }
    if(value > middle_value+signal_width){
        value = middle_value+signal_width;
    }
    value -= middle_value;

    // How much the motors react to the IR sensor
    float responsiveness =2.5;// 2.25;
    value *= responsiveness;
    //Printf("%f\n", value);

    // How fast the robot drives straight
    float speed = 0.5;//55;

    Printf("@%f\n", (speed+value));
    // Adjust motor speeds
    SetMotor(left_motor, coerce(speed-value, 0, 0.99));
    SetMotor(right_motor, coerce(speed+value, 0, 0.99));
}

int main(void) {

    // Initialize IR sensor
    ir_sensor = InitializeADC(PIN_D0);

    // Initialize RGB LEDs
    green = InitializePWM(PIN_F3, 500);
    red = InitializePWM(PIN_F1, 500);
    blue = InitializePWM(PIN_F2, 500);
    initialize_led_variables();

    // Initialize motors
    left_motor = InitializeServoMotor(PIN_E3, false);
    right_motor = InitializeServoMotor(PIN_E2, true);

    // Start LED color-changing
    //CallEvery(change_led_pwm, 0, light_refresh_step);

    // Start line following
    //ADCBackgroundRead(ir_sensor, adjust_motor_power, NULL);

    //SetMotor(left_motor, 0.5);
    //SetMotor(right_motor, 0.5);
    //CallEvery(adjust_motor_power, 0, 0.1);
    // Infinite loop so that program keeps running
    while(1){
    adjust_motor_power();
    //Printf("Hello World!\n");
    }
}
