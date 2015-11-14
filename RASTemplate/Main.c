/* Main.h
 *
 * This is the source code for the robot "Ultron" built for
 * The University of Texas at Austin's IEEE RAS fall 2015
 * Robotathon competition.
 *
 * The robot has a left and right drive motor, two intake motors,
 * and one marble/ping-pong ball release motor. It also uses two
 * IR distance sensors and a line sensing phototransistor array.
 *
 * This program assumes the robot is placed in front of its goal
 * facing right with the line sensing array behind the black
 * line. It then drives across the line to calibrate the array,
 * then begins a scoring loop in which it follows the wall on the
 * right-hand side with the intake runing, and pauses to score
 * whenever the line sensor array detects a goal.
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

/* define io components */
tMotor * left_motor;
tMotor * right_motor;
tMotor * in_motor;
tMotor * brush_motor;
tServo * release_servo;

tADC * side_ir_sensor;
tADC * front_ir_sensor;
tLineSensor * line_sensor;

tPWM * red_led;
tPWM * green_led;
tPWM * blue_led;

/* define pin connections */
#define left_motor_pin PIN_B7
#define right_motor_pin PIN_B6
#define in_motor_pin PIN_A4
#define brush_motor_pin PIN_E0
#define release_servo_pin PIN_B2

#define side_ir_sensor_pin PIN_E4
#define front_ir_sensor_pin PIN_E5
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




/* helper function to fit a value between min and max */
float coerce(float val, float min, float max){
    if(val < min){
        val=min;
    }
    if(val > max){
        val=max;
    }
    return val;
}

/* adjusts motor speeds based on light sensor readings */
void adjust_motor_power(){

    /* get values from ir sensors */
    float side_value = ADCRead(side_ir_sensor);
    float front_value = ADCRead(front_ir_sensor);

    /* table of ir values for different distances */
    /* 12"  .3  */
    /* 9"   .35 */
    /* 6"   .54 */
    /* 5"   .62 */
    /* 4"   .74 */
    /* 3"   .87 */
    /* 2"   .94 */
    /* 1"   (fluctuates) */
    /* end of table */

    #define USE_OLD_CODE
    /* old line following code */
    #ifdef USE_OLD_CODE
        // The ideal IR sensor value
        float middle_value = 0.5;
        float signal_width = 0.3;
        if(side_value < middle_value-signal_width){
            side_value = middle_value-signal_width;
        }
        if(side_value > middle_value+signal_width){
            side_value = middle_value+signal_width;
        }
        side_value -= middle_value;
        // How much the motors react to the IR sensor
        float responsiveness =1.5;// 2.25;
        side_value *= responsiveness;
        //Printf("%f\n", value);
        // How fast the robot drives straight
        float speed = 0.5;//55;
    #endif

    Printf("%f - %f\n", side_value, front_value);

    // Adjust motor speeds
    if(front_value > 0.35){ // Turn left if there is a wall in front
        SetMotor(left_motor, -1);
        SetMotor(right_motor, 1);
    }else{ // Else, follow the wall using side sensor
        SetMotor(left_motor, coerce(speed-side_value, 0, 1));
        SetMotor(right_motor, coerce(speed+side_value, 0, 1));
    }
} /* end of adjust_motor_power function */

void follow_line(){
    float array[8];
    LineSensorReadArray(line_sensor, array);
    float value = array[0];
    //Printf("%f",array[4]);
    value *= 10;
    value = 1-value;
    //value *= 10;

    SetPWM(red, value, 0);

    if(value > 0.5){
        SetMotor(left_motor, 0);
        SetMotor(right_motor, -1);
}else{
SetMotor(left_motor, -1);
SetMotor(right_motor, 0);}
    //SetMotor(left_motor, value);
    //SetMotor(right_motor, value);
}

void blink(void){
SetPin(PIN_F1, led);
led=!led;
   }

/* The program's main function */
int main(void) {

    /* initialize motors */
    left_motor = InitializeServoMotor(PIN_E3, false);
    right_motor = InitializeServoMotor(PIN_E2, true);
    in_motor = InitializeServoMotor(PIN_E1, true);
    brush_motor = InitializeServoMotor(PIN_D1, true);
    SetMotor(in_motor, 1);
    SetMotor(brush_motor, 1);

    /* initialize ir sensors */
    front_ir_sensor = InitializeADC(PIN_D0);
    side_ir_sensor = InitializeADC(PIN_D3);

    /* initialize line sensor */
    line_sensor = 
InitializeGPIOLineSensor(PIN_A3,PIN_A4,PIN_B6,PIN_B7,PIN_C6,PIN_F0,PIN_C7,PIN_B2);
    SetPin(PIN_A2, 1);

    /* turn on light code if needed */
    #ifdef LIGHTS
        /* initialize rgb leds */
        //green = InitializePWM(PIN_F3, 500);
        red = InitializePWM(PIN_F1, 500);
        //blue = InitializePWM(PIN_F2, 500);
    #endif

    //#define MOTOR_TEST
    /* drive motors straight if needed */
    #ifdef MOTOR_TEST
        SetMotor(left_motor, 1);
        SetMotor(right_motor, 1);
    #endif

//    CallEvery(blink,0,1);

    /* infinite loop that program runs inside of */
    while(1){
      follow_line();
  //    Printf("Hi");
    }
   
} /* end of main function */

/* ========== end of program ========== */
