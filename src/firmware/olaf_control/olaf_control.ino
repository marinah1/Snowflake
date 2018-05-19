/* Firmware for IARRC 2017
 * Author: Vincent Yuan,
 * Modified: Nick Wu
 * Modified: James Asefa
 * Modified: Jinhao Lu
 * Modified: Gareth Ellis
 * Modified: Valerian Ratu
 * Modified: Marinah Zhao
 * Date Last Modified: April 7th, 2018
 */

/*
 * UBC Snowbots - IARRC 2017
 * Firmware for Control of an RC car
 *
 * This firmware will take in a message of the form:
 * `B<linear x><linear y><linear z><angular x><angular y><angular z>`
 * where each `<>` is a single byte. The degree to rotate the servo controlling
 * the front wheels is determined from `angular z`, and the speed of the car by `linear x`
 * (the rest of the values are discarded)
 */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <Servo.h>
#include <math.h>
#include <std_msgs/String.h>

// !!!!WARNING!!!!
// DEBUGGING might affect performance and alter normal behaviour
// !!!!WARNING!!!!
// Uncomment this to enable debug message logging - messages sent over serial will be echoed back
//#define DEBUG_MESSAGES
// Uncomment this to enable PWM debug logging - PWM signals will be echoed
//#define DEBUG_PWM
#define DEBUG_SERIAL

#define BAUD_RATE 9600

// size of character buffer being passed over serial connection
#define BUFFER_SIZE 7

// Robot will keep executing the last command unless the period exceeds the SAFETY_TIMEOUT
// SAFETY_TIMEOUT is in ms
#define SAFETY_TIMEOUT 500

void serial_read();
void convert();
void drive(int, int);

// buffer inputs
double linear_x = 0.0;
double linear_y = 0.0;
double linear_z = 0.0;
double angular_x = 0.0;
double angular_y = 0.0;
double angular_z = 0.0;

// motor pins
const int MOTOR_PIN = A0;
const int DIRECTION_PIN = A2;

// max and min linear speeds and stopping condition
const double LINEAR_SPEED_MAX = 50.0;
const double LINEAR_SPEED_MIN = -50.0;

// max and min angular speeds and stopping condition
const double ANGULAR_SPEED_MAX = 50.0;
const double ANGULAR_SPEED_MIN = -50.0;

// max and min wheel angles and stopping condition
const int WHEEL_ANGLE_MAX = 25;
const int WHEEL_ANGLE_MIN = -25;    //-23 actual value, made symmetrical
const int WHEEL_ANGLE_STOP = 0;

// length between axis of front and back wheels in m
const double WHEEL_BASE_LENGTH = 0.28;

// max and min radius of turn calculated from wheel constants in m
//  calculated with radius = wheel_base_length / sin(wheel_angle)
//  e.g. WHEEL_TURN_RADIUS_MAX = WHEEL_BASE_LENGTH / sin(WHEEL_ANGLE_MAX)
//  referenced: https://github.com/AutoRally/autorally/wiki/Wheel-Odometry
const double WHEEL_TURN_RADIUS_MAX = 0.66;
const double WHEEL_TURN_RADIUS_MIN = -0.66;   //-72 actual value, made symmetrical

// max and min PWM of servo motor given max and min wheel angles
const int WHEEL_PWM_MAX = 2000;      //TODO
const int WHEEL_PWM_MIN = 1000;     //TODO
const int WHEEL_PWM_STOP = 1500;     //TODO

// The minimum and maximum PWM signals to map received values to
// This is the default for both the Servo and the Talon ESC
// 1000us - 2000us
const int MIN_PWM = 1000;
const int MAX_PWM = 2000;

// The safety cutoffs to prevent us drawing too much current
// from the motor
const int MIN_MOTOR_PWM_CUTOFF = 1300;
const int MAX_MOTOR_PWM_CUTOFF = 1700;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

Servo motor;
Servo direction_motor;

ros::NodeHandle nh;

#ifdef DEBUG_SERIAL
  std_msgs::String debug_pub_msg;
  ros::Publisher debug_pub("olaf_arduino", &debug_pub_msg);
#endif

void TwistCb( const geometry_msgs::Twist& twist_msg){

    previousMillis = millis();

    linear_x = twist_msg.linear.x;
    angular_z = twist_msg.angular.z;

#ifdef DEBUG_SERIAL
    String msg = "linear_x: " + String(linear_x) + ", angular_z: " + String(angular_z);
    char charmsg[100];
    msg.toCharArray(charmsg, 100);
    debug_pub_msg.data = charmsg;
    debug_pub.publish(&debug_pub_msg);
#endif

    convert();
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &TwistCb );

void setup() {
    Serial.begin(57600);

    // Initialiase the Servo and Motor connections
    motor.attach(MOTOR_PIN, MIN_PWM, MAX_PWM);
    direction_motor.attach(DIRECTION_PIN, MIN_PWM, MAX_PWM);

    if (!checkWheelConstants()) {
        while(1);
    }

    nh.initNode();
    nh.subscribe(sub);

#ifdef DEBUG_SERIAL
    nh.advertise(debug_pub);
#endif

}

void loop() {
    nh.spinOnce();
    check_timeout();
    drive(linear_x, angular_z);
}

void check_timeout() {
    // if havent received twist message in a while, stop moving robot
    currentMillis = millis();
    if(currentMillis - previousMillis > SAFETY_TIMEOUT){
        linear_x = 0.0;
    }
}

void convert() {
    // safety check if values are out of range
    if (linear_x > LINEAR_SPEED_MAX)
        linear_x = LINEAR_SPEED_MAX;
    else if (linear_x < LINEAR_SPEED_MIN)
        linear_x = LINEAR_SPEED_MIN;

    if (angular_z > ANGULAR_SPEED_MAX)
        angular_z = ANGULAR_SPEED_MAX;
    else if (angular_z < ANGULAR_SPEED_MIN)
        angular_z = ANGULAR_SPEED_MIN;
}

int checkWheelConstants() {

    // Check if (WHEEL_BASE_LENGTH/radius) must be within [-1,1]
    if ((WHEEL_BASE_LENGTH/WHEEL_TURN_RADIUS_MAX > 1.0) ||
            (WHEEL_BASE_LENGTH/WHEEL_TURN_RADIUS_MIN < -1.0)) {

#ifdef DEBUG_MESSAGES
        Serial.println("Wheel constants are not accurate.");
#endif
        return 0;
    }
    return 1;
}

int get_wheel_PWM(double linear_speed, double angular_speed) {

    // Assumes linear speed is in m/s and angular speed is in rad/s
    if (angular_speed == 0 || linear_speed == 0) {
        return WHEEL_PWM_STOP;
    }
    double radius = linear_speed / angular_speed;

    // Checks if radius is within range of wheel turning capabilities
    if (radius > WHEEL_TURN_RADIUS_MAX) {
        radius = WHEEL_TURN_RADIUS_MAX;
    } else if (radius < WHEEL_TURN_RADIUS_MIN) {
        radius = WHEEL_TURN_RADIUS_MIN;
    }

    // Ensures asin will not produce nan (produces [-pi/2, pi/2])
    double ratio = WHEEL_BASE_LENGTH / radius;
    if (ratio > 1.0) {
        ratio = 1.0;
    } else if (ratio < -1.0) {
        ratio = -1.0;
    }

    int wheel_angle = RAD_TO_DEG*asin(ratio);

    int angle_pwm = map(wheel_angle, WHEEL_ANGLE_MIN, WHEEL_ANGLE_MAX, WHEEL_PWM_MIN, WHEEL_PWM_MAX);

    return angle_pwm;
}

int pid_controller(int linear_speed){

    // TODO: Stand in function
    // PID NOT YET IMPLEMENTED

    int velocity = map(linear_speed, LINEAR_SPEED_MIN, LINEAR_SPEED_MAX, MIN_PWM, MAX_PWM);

    // Check that we're not outside our safety cutoffs
    if (velocity > MAX_MOTOR_PWM_CUTOFF)
        velocity = MAX_MOTOR_PWM_CUTOFF;
    else if (velocity < MIN_MOTOR_PWM_CUTOFF)
        velocity = MIN_MOTOR_PWM_CUTOFF;

    return velocity;

#ifdef DEBUG_PWM

#endif
}

void drive(double linear_speed, double angular_speed){

    // Default velocity and angle to the midpoint in the PWM range (STOP value)
    int angle_PWM = WHEEL_PWM_STOP;
    int velocity = (MAX_PWM - MIN_PWM)/2;

    angle_PWM = get_wheel_PWM(linear_speed, angular_speed);
    velocity = pid_controller(linear_speed);

    // Write the commands to the motor and the servo
    motor.writeMicroseconds(velocity);
    direction_motor.writeMicroseconds(angle_PWM);

#ifdef DEBUG_PWM
    Serial.print("Input linear speed: ");
    Serial.print(linear_speed);
    Serial.print(" / ");
    Serial.print("Input angular speed: ");
    Serial.print(angular_speed);
    Serial.print(" / ");
    Serial.print("Angle PWM");
    Serial.print(angle_PWM);
    Serial.print(" / ");
    Serial.println();
#endif
}