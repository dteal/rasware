#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/pwm.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/adc.h>
#include <RASLib/inc/linesensor.h>
#include <math.h>

int led = 1;

/* define led objects if necessary */
#define LIGHTS
#ifdef LIGHTS
    tPWM * green;
    tPWM * red;
    tPWM * blue;
#endif

/* define io components */
tMotor * left_motor;
tMotor * right_motor;
tMotor * in_motor;
tMotor * brush_motor;
tADC * side_ir_sensor;
tADC * front_ir_sensor;
tLineSensor * line_sensor;
tPin * line_led_pin;

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
