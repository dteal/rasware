/* Main.h
 *
 * This is test source code for the robot "Ultron" built for
 * The University of Texas at Austin's IEEE RAS fall 2015
 * Robotathon competition.
 *
 * The robot has a left and right drive motor, two intake motors,
 * and one marble/ping-pong ball release motor. It also uses three
 * IR distance sensors.
 *
 * THIS IS NOT THE COMPETITION CODE!
 * THIS IS FOR RUNNING CHECKPOINTS.
 *
 * Authors: Yazan Alatrach, Angelique Bautista, Daniela Barrios,
 *          Angelique Bautista, Shrikar Murthy, and Daniel Teal.
 * UT RAS Mentor: CJ Pleiter
 */

#include <RASLib/inc/adc.h>
#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/linesensor.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/pwm.h>
#include <RASLib/inc/servo.h>
#include <RASLib/inc/time.h>
#include <math.h>

// define io components
tMotor * left_motor;
tMotor * right_motor;
tMotor * in_motor;
tMotor * brush_motor;
tServo * release_servo;

tADC * side_ir_sensor;
tADC * front_ir_sensor;
tADC * goal_ir_sensor;

//tPWM * speaker;

// define pin connections
#define left_motor_pin PIN_B7
#define right_motor_pin PIN_B6
#define in_motor_pin PIN_A4
#define brush_motor_pin PIN_A3
#define release_servo_pin PIN_B2

#define side_ir_sensor_pin PIN_E5
#define front_ir_sensor_pin PIN_E4
#define goal_ir_sensor_pin PIN_B4

#define red_led_pin PIN_F1
#define green_led_pin PIN_F3
#define blue_led_pin PIN_F2

#define speaker_pin PIN_D3

// magic numbers
float side_max = 0.93; // distance to side wall
float front_max = 0.7; // distance to front wall
float goal_max = 0.3; // difference in between goal/wall distance
#define AVG_LEN 1000 // number of measurements averaged
#define WAIT_FOR_BUTTON_TO_START

// global variables
int running = 0; // tracks state of program
int reset_ir_flag = 1; // tracks ir averaging restarts
float front_avg[AVG_LEN] = {}; // averages front ir sensor values
float side_avg[AVG_LEN] = {}; // averages side ir sensor values
float goal_avg[AVG_LEN] = {}; // averages goal ir sensor values
int avg_idx = 0; // keepts track of current position in averaging arrays

// function forward declarations
void initialize_pins(); // initializes io components
void react_to_button(); // triggers on button press
float coerce(float n, float min, float max, float val);
float average(float * array, int length); // averages array
void fill_array(float * array, int length, float value); // fills array

// main function
int main(void){
    // initialize io
    initialize_pins();

    // set up sensors
    ADCReadContinuously(side_ir_sensor, 0.01);
    ADCReadContinuously(front_ir_sensor, 0.01);
    ADCReadContinuously(goal_ir_sensor, 0.01);

    // turn on status led
    SetPin(green_led_pin, 1);

    #ifdef WAIT_FOR_BUTTON_TO_START
        // wait for button press
        CallOnPinRising(react_to_button, 0, PIN_F0);
        while(!running){}
    #else
        running=1;
    #endif

    // main loop
    while(1){
        if(running){
            // poll sensors
            float side_value = ADCRead(side_ir_sensor);
            float front_value = ADCRead(front_ir_sensor);
            float goal_value = ADCRead(goal_ir_sensor);

            // sanitize values
            side_value = coerce(side_value, 0, 1, side_avg[avg_idx]);
            front_value = coerce(front_value, 0, 1, front_avg[avg_idx]);
            goal_value = coerce(goal_value, 0, 1, goal_avg[avg_idx]);

            float side_dist = 0;
            float front_dist = 0;
            float goal_dist = 0;
            if(reset_ir_flag){
                // initialize averaging arrays to values
                fill_array(side_avg, AVG_LEN, side_value);
                fill_array(front_avg, AVG_LEN, front_value);
                fill_array(goal_avg, AVG_LEN, goal_value);
                reset_ir_flag = 0;
                side_dist = side_value;
                front_dist = front_value;
                goal_dist = goal_value;
            }else{
                // average values
                avg_idx++;
                if(avg_idx>=AVG_LEN){
                    avg_idx=0;
                }
                side_avg[avg_idx] = side_value;
                front_avg[avg_idx] = front_value;
                goal_avg[avg_idx] = goal_value;
                side_dist = average(side_avg, AVG_LEN);
                front_dist = average(front_avg, AVG_LEN);
                goal_dist = average(goal_avg, AVG_LEN);
            }

            // follow wall
            if(side_value > side_max || front_value > front_max){
                SetMotor(left_motor, -1);
            }else{
                SetMotor(left_motor, 1);
            }
            SetMotor(right_motor, 1);
        }
    }
}

// intialize io objects
void initialize_pins(){
    left_motor = InitializeServoMotor(left_motor_pin, false);
    right_motor = InitializeServoMotor(right_motor_pin, true);
    //in_motor = InitializeServoMotor(in_motor_pin, true);
    //brush_motor = InitializeServoMotor(brush_motor_pin, true);
    //release_servo = InitializeServo(release_servo_pin);

    side_ir_sensor = InitializeADC(side_ir_sensor_pin);
    front_ir_sensor = InitializeADC(front_ir_sensor_pin);
    goal_ir_sensor = InitializeADC(goal_ir_sensor_pin);
}

// start running when button is pressed
void react_to_button(){
    running = 1;
}

// makes n val if n not in [min, max]
float coerce(float n, float min, float max, float val){
    if(n < min || n > max){
        n = val;
    }
    return n;
}

// averages values in array
float average(float * array, int length){
    float count = 0;
    for(int i = 0; i < length; i++){
        count += array[i];
    }
    count /= length;
    return count;
}

// fills array with given value
void fill_array(float * array, int length, float value){
    for(int i = 0; i < length; i++){
        array[i] = value;
    }
}

// end of program
