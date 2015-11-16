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
 * whenever the third IR sensor detects a goal.
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

tPin * red_led;
tPWM * green_led;
tPWM * blue_led;

// define pin connections
#define left_motor_pin PIN_B7
#define right_motor_pin PIN_B6
#define in_motor_pin PIN_A4
#define brush_motor_pin PIN_A3
#define release_servo_pin PIN_B2

#define side_ir_sensor_pin PIN_E5
#define front_ir_sensor_pin PIN_E4
#define goal_ir_sensor_pin PIN_B4
#define ls_pin_1 PIN_B4
#define ls_pin_2 PIN_E3
#define ls_pin_3 PIN_E2
#define ls_pin_4 PIN_E1
#define ls_pin_5 PIN_D3
#define ls_pin_6 PIN_D2
#define ls_pin_7 PIN_D1
#define ls_pin_8 PIN_D0

#define red_led_pin PIN_F1
#define green_led_pin PIN_F3
#define blue_led_pin PIN_F2

// magic numbers
float release_marbles = 1; // servo position to release marbles
float release_pp_balls = 0.25; // position to release pp balls
float release_nothing = 0.8; // position to hold on to both
float side_max = 0.93; // distance to side wall
float front_max = 0.7; // distance to front wall
float goal_max = 0.2; // difference in between goal/wall distance
#define AVG_LEN 1000 // number of measurements averaged

// global variables
int running = 0; // tracks state of program
int side = 0; // tracks current side of field
long time = 0;
float front_avg[AVG_LEN] = {};
float side_avg[AVG_LEN] = {};
float goal_avg[AVG_LEN] = {};
int avg_idx = 0;

// function forward declarations
void initialize_pins(); // initializes io components
void react_to_button(); // triggers on button press
void dispense(); // scores balls in goal
float coerce(float n, float min, float max, float val);
float average(float * array, int length);

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

    // wait for button press
    CallOnPinRising(react_to_button, 0, PIN_F0);
    while(!running){}

    // turn on motors
    SetServo(release_servo, release_nothing);
    SetMotor(in_motor, 1);
    SetMotor(brush_motor, 1);

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

            // average values
            avg_idx++;
            if(avg_idx>=AVG_LEN){
                avg_idx=0;
            }
            side_avg[avg_idx] = side_value;
            front_avg[avg_idx] = front_value;
            goal_avg[avg_idx] = goal_value;
            float side_dist = average(side_avg, AVG_LEN);
            float front_dist = average(front_avg, AVG_LEN);
            float goal_dist = average(goal_avg, AVG_LEN);

            if(!(time%100)){
              Printf("IR:\t%f\t%f\t%f\n",side_dist,front_dist,goal_dist);
            }

            // score or follow wall
            if(side_value - goal_value > goal_max){
                SetPin(blue_led_pin, 0);
                if(side){
                    SetPin(green_led_pin, 0);
                    SetPin(red_led_pin, 1);
                }else{
                    SetPin(green_led_pin, 1);
                    SetPin(red_led_pin, 0);
                }
                side = !side;
            }else{
                if(side_value > side_max || front_value > front_max){
                    SetMotor(left_motor, -1);
                }else{
                    SetMotor(left_motor, 1);
                }
                SetMotor(right_motor, 1);
            }
        time++;
        }
    }
}

void dispense(){
    // change leds to incidate state
    SetPin(green_led_pin, 0);
    SetPin(red_led_pin, 0);
    SetPin(blue_led_pin, 1);

    // drive to center of goal
    SetMotor(left_motor, 1);
    SetMotor(right_motor, 1);
    //Wait(0.5);

    // turn off intake and brush
    SetMotor(in_motor, 0);
    SetMotor(brush_motor, 0);

    // turn left
    SetMotor(left_motor,-1);
    SetMotor(right_motor,1);
    //Wait(1.5);

    // release balls
    SetMotor(left_motor, 0);
    SetMotor(right_motor, 0);
    if(side){
        //SetServo(release_servo, release_marbles);
    }else{
        //SetServo(release_servo, release_pp_balls);
    }
    //Wait(3);
    //SetServo(release_servo, release_nothing);

    // turn right
    SetMotor(left_motor, 1);
    SetMotor(right_motor,-1);
    //Wait(1.5);

    // drive past goal
    SetMotor(left_motor, 1);
    SetMotor(right_motor, 1);
    SetMotor(in_motor, 1);
    SetMotor(brush_motor, 1);
    //Wait(2);

    side = !side;
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

float average(float * array, int length){
    float count = 0;
    for(int i = 0; i < length; i++){
        count += array[i];
    }
    count /= length;
    return count;
}

/* end of program */
