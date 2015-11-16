/* Main.h
 *
 * This is the source code for the robot "Ultron" built for
 * The University of Texas at Austin's IEEE RAS fall 2015
 * Robotathon competition.
 *
 * The robot has a left and right drive motor, two intake motors,
 * and one marble/ping-pong ball release motor. It also uses three
 * IR distance sensors.
 *
 * This program assumes the robot is placed in front of its goal
 * facing right. It runs a loop in which it follows the wall on
 * the right-hand side with the intake runing, and pauses to score
 * whenever the third IR sensor detects a goal by its difference from
 * the normal side IR sensor.
 *
 * Sensor values are continually averaged for more reliable values.
 * Additionally, several sanity checks are in place: after a goal is
 * detected, it must be present (on average) for some time before
 * triggering a scoring response. This avoids false positives. Also,
 * there is a minimum time between different goals that must be met.
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
float release_marbles = 1; // servo position to release marbles
float release_pp_balls = 0.25; // position to release pp balls
float release_nothing = 0.8; // position to hold on to both
float side_max = 0.93; // distance to side wall
float front_max = 0.7; // distance to front wall
float goal_max = 0.3; // difference in between goal/wall distance
#define AVG_LEN 1000 // number of measurements averaged
float goal_min_time = 10; // minimum time between goals
float goal_drive_time = 2; // time to drive to middle of goal
//#define WAIT_FOR_BUTTON_TO_START

// global variables
int running = 0; // tracks state of program
int side = 0; // tracks current side of field
int reset_ir_flag = 1; // tracks ir averaging restarts
float front_avg[AVG_LEN] = {}; // averages front ir sensor values
float side_avg[AVG_LEN] = {}; // averages side ir sensor values
float goal_avg[AVG_LEN] = {}; // averages goal ir sensor values
int avg_idx = 0; // keepts track of current position in averaging arrays
float last_score_time = 0; // time since last goal
float goal_detect_time = 0; // time since potential goal detected
float goal_value_sum = 0; // sum of goal readings since goal detected
int num_goal_values = 0; // number of goal readings in that time
int in_goal = 0; // whether robot is possibly in front of goal

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

    //speaker = InitializePWM(speaker_pin, 600);
    //SetPWM(speaker, 0.5, 0);

    #ifdef WAIT_FOR_BUTTON_TO_START
        // wait for button press
        CallOnPinRising(react_to_button, 0, PIN_F0);
        while(!running){}
    #else
        running=1;
    #endif

    // turn on motors
    SetServo(release_servo, release_nothing);
    SetMotor(in_motor, 1);
    SetMotor(brush_motor, 1);

    // initialize timing variable
    last_score_time = GetTime();

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

            if(in_goal){
                goal_value_sum += (side_value - goal_value);
                num_goal_values++;
            }

            // detect potential goal
            if(!in_goal && side_value - goal_value > goal_max && 
GetTime() - last_score_time > goal_drive_time){
                goal_detect_time = GetTime();
                in_goal = 1;
                goal_value_sum = 0;
                num_goal_values = 0;
                // set status leds
                SetPin(red_led_pin, 0);
                SetPin(green_led_pin, 0);
                SetPin(blue_led_pin, 1);
            }

            if(in_goal && GetTime() - goal_detect_time > 
goal_drive_time){ // score
                if(goal_value_sum/num_goal_values > goal_max){
                    // turn off intake
                    SetMotor(in_motor, 0);
                    SetMotor(brush_motor, 0);
                    // turn left
                    SetMotor(left_motor, -1);
                    SetMotor(right_motor, 1);
                    Wait(1.5);
                    // drive away from goal
                    SetMotor(in_motor, 1);
                    //SetMotor(brush_motor, 1);
                    SetMotor(left_motor, 1);
                    SetMotor(right_motor, 1);
                    Wait(2);
                    // open hatch, run into goal
                    SetMotor(in_motor, 0);
                    //SetMotor(brush_motor, 0);
                    SetMotor(left_motor, -1);
                    SetMotor(right_motor, -1);
                    if(side){
                        SetServo(release_servo, release_marbles);
                    }else{
                        SetServo(release_servo, release_pp_balls);
                    }
                    Wait(2.5);
                    // drive back away from goal
                    SetMotor(left_motor, 1);
                    SetMotor(right_motor, 1);
                    Wait(0.5);
                    SetServo(release_servo, release_nothing);
                    // turn right
                    SetMotor(left_motor, 1);
                    SetMotor(right_motor, -1);
                    Wait(1.5);
                    // turn intake on and continue
                    SetMotor(in_motor, 1);
                    SetMotor(brush_motor, 1);
                    side = !side;
                    last_score_time = GetTime();
                }
                // set status leds
                SetPin(blue_led_pin, 0);
                if(side){
                    SetPin(green_led_pin, 0);
                    SetPin(red_led_pin, 1);
                }else{
                    SetPin(green_led_pin, 1);
                    SetPin(red_led_pin, 0);
                }
                in_goal = 0;
            }else{ // follow wall
                if(side_value > side_max || front_value > front_max){
                    SetMotor(left_motor, -1);
                }else{
                    SetMotor(left_motor, 1);
                }
                SetMotor(right_motor, 1);
            }
        }
    }
}

// intialize io objects
void initialize_pins(){
    left_motor = InitializeServoMotor(left_motor_pin, false);
    right_motor = InitializeServoMotor(right_motor_pin, true);
    in_motor = InitializeServoMotor(in_motor_pin, true);
    brush_motor = InitializeServoMotor(brush_motor_pin, true);
    release_servo = InitializeServo(release_servo_pin);

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
