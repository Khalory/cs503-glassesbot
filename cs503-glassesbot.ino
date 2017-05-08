#include "Wire.h"                 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "DualMC33926MotorShield.h"
#define MPU_INT 2//0
#define PWM_L 10
#define PWM_R 11
#define DIR_L1 6
#define DIR_L2 12
#define DIR_R1 8
#define DIR_R2 9
#define SPD_INT_L 3//1
#define SPD_PUL_L 4
#define SPD_INT_R 4//7
#define SPD_PUL_R 5
// Left encoder
#define IR_1 2
#define IR_2 6
// Right encoder
#define IR_3 3
#define IR_4 5
// Pins 2 and 3 correspond to interrupts 0 and 1, respectively
#define IR_INTERRUPT_1 0
#define IR_INTERRUPT_2 1
//Ping constants
const int pingPinRight = 13;
const int pingPinFront = 11;
const int MinRightDist = 2;
const int MaxRightDist = 8;
const int MinFrontDist = 10;
const int MaxFrontDist = 150;
#define NUM_PINGS 5
double pings[NUM_PINGS];
double avgPingChange = 0.0d;
double avgPingFlow = 0.4d; // Smoothing factor
// These variables will keep track of our current state
bool turnLeft = false;
bool turnRight = false;
uint8_t turnRightState = 0;
//counters to check is we should switch from our current state
int checkTurnRight = 0;
const int checkTurnRightMax = 20; // needs to be calibrated
int checkTurnLeft = 0;
const int checkTurnLeftMax = 20;
int checkStraight = 0;
const int checkStraightMax = 20;
// Stores the total number of steps for both encoders
volatile long leftEncoder = 0;
volatile long rightEncoder = 0;
// Tracks the number of steps since the last world position update
volatile int leftSteps = 0;
volatile int rightSteps = 0;
const int stepsPerUpdate = 1;
// MOTION CONTROLLER CODE (JUST THE ONE LINE BELOW)
const float max_phi = 0.02f;// Max angle offset of robot
const float max_dist = 0.7f;
const float botRadius = 4.125f;
const float wheelRadius = 1.39f;
const float stepDistance = 2.0f * wheelRadius * PI / 16.0f; // 2PI*r / 16 steps
// World Coordinates
float worldX = 0.0f;  
float worldY = 0.0f;
float worldTheta = 0.0f;
// MOTION CONTROLLER CODE (BELOW)
// Previous World Coordinates
const uint8_t velocityUpdateCount = 10;
const uint8_t velocityUpdateStep = 10;
float prevWorldXs[velocityUpdateCount];
float prevWorldYs[velocityUpdateCount];
float prevWorldThetas[velocityUpdateCount];
float prevWorldTimes[velocityUpdateCount];
float velocityUpdateTimer = millis();
// Current Velocity
float xVelocity = 0.0f;
float yVelocity = 0.0f;
float thetaVelocity = 0.0f;
// Goal location of the current goal we are trying to reach
float goalLocationX = 0.0f;
float goalLocationY = 0.0f;
float goalLocationTheta = 0.0f;
const int numPoints = 50;
const float curveAngle = PI;
float goalXs[numPoints];
float goalYs[numPoints];
float translate_velocity = 0.0f;
float turn_velocity = 0.0f;
float K = 8.0f;
float B = 0.13f;
float K_t = 0.008f; //0.008f;
float B_t = 0.008f; //0.01f;
float K_r = 0.55f; //0.15f
float B_r = 0.175f; //0.175f
float K_w = 0.00125f;
float B_w = 0.01f;
float angle_ref =- 0.005f; // angle_ref is center of gravity offset
float angular_rate_ref = 0.0f;
long time_gain_scaler = micros();
long global_time = micros();
float left_min_motor_speed = 30;
float right_min_motor_speed = 30;
float pwm_left = 0.0f;
float pwm_right = 0.0f;
MPU6050 mpu;
int16_t ax;
int16_t ay;
int16_t az;
int16_t gx;
int16_t gy;
int16_t gz;
double pitch;
double timer = millis();
// Motor driver
DualMC33926MotorShield md;
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIREI
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  init_IO();
  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // initialize devices
  mpu.initialize();
  md.init();
  init_IR();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  
//  generateGoals(goalXs, goalYs);
}
void loop() {
  if (millis() - timer > 0) {
    getValuesFromPing();
    timer = millis();
  }
  motionControllerCode();
}
void motionControllerCode() {
  if (shouldUpdateCoords())
    updateCoords();
  if (millis() - velocityUpdateTimer > velocityUpdateStep)
    updateVelocity();
  //updateGoal();
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   
  getPitch(&pitch, &ax, &az);
  // Translate Controller
  float distance_to_goal = pow((pow(goalLocationX - worldX, 2) + pow(goalLocationY - worldY, 2)), .5);
  double direction_to_goal = atan2(goalLocationY - worldY, goalLocationX - worldX);
  double theta_difference = clampThetaDifference(worldTheta - direction_to_goal);
  if (abs(theta_difference) > PI/2) {
    distance_to_goal *= -1;
  }
  // Bound our distance within [-max_dist, max_dist]
  distance_to_goal = max(-max_dist, min(max_dist, distance_to_goal));

  // turnRightState 1 works by using the actual goalLocations
  if(!(turnLeft || turnRight) || turnRightState == 3)
    distance_to_goal = -max_dist;
  // turnRightState 2 is the one which only turns by PI/2
  if(turnLeft || turnRightState == 2)
    distance_to_goal = 0;
  
  double direction_of_velocity = atan2(yVelocity, xVelocity);
  double velocity = pow((pow(xVelocity, 2) + pow(yVelocity, 2)), .5);
  theta_difference = clampThetaDifference(worldTheta - direction_of_velocity);
  if (abs(theta_difference) > PI/2) {
    velocity *= -1;
  }

  float vel = (pwm_left+pwm_right)*0.5;
  double delta_phi = K_t * distance_to_goal - B_t * (vel-translate_velocity);
  delta_phi = max(-max_phi, min(max_phi, delta_phi));
  double angle_err = pitch - angle_ref + delta_phi; // angle_ref is center of gravity offset
  double angular_rate = PI*((double)gy)/180 - angular_rate_ref; // Converted to radians
  // Balance Controller
  float deltaPWM = -K * angle_err - B * angular_rate;

  // We are going straight
  if (!(turnRight || turnLeft)) {
    double dist = (pings[0] + pings[1])*.5 - 3.5f;
    //theta_err = K_w*dist + B_w * avgPingChange * (isMovingForward() ? -1 : 1);
    goalLocationTheta += K_w*dist + B_w * avgPingChange * (isMovingForward() ? -1 : 1);
    
    //Serial.print(theta_err);
//    Serial.print(" = K(");
//    Serial.print(K_w*dist);
//    Serial.print(") + B(");
//    Serial.print(B_w * avgPingChange * (isMovingForward() ? -1 : 1));
//    Serial.println(")");
  }
//  Serial.println(theta_err);
  Serial.print(turnLeft);
  Serial.print("  ");
  Serial.print(turnRight);
  Serial.print("  ");
  Serial.println(turnRightState);

  // Rotate Controller
  float theta_err = clampThetaDifference(goalLocationTheta - worldTheta);
  
  long timeDiff = micros() - time_gain_scaler;
  time_gain_scaler = micros();
  
  thetaVelocity = PI*((double)gz)/180; // Converted to radians
  float deltaPWMR = K_r * theta_err - B_r * thetaVelocity;
  float nextPwm = pwm_left + (deltaPWM + deltaPWMR)*((float)timeDiff)*0.001;
  if (pwm_left > left_min_motor_speed && nextPwm < left_min_motor_speed) {
    nextPwm -= 2*left_min_motor_speed;
  }
  else if (pwm_left < -left_min_motor_speed && nextPwm > -left_min_motor_speed) {
    nextPwm += 2*left_min_motor_speed;
  }
  
  pwm_left = nextPwm;
  nextPwm = pwm_right + (deltaPWM - deltaPWMR)*((float)timeDiff)*0.001;
  if (pwm_right > right_min_motor_speed && nextPwm < right_min_motor_speed) {
    nextPwm -= 2*right_min_motor_speed;
  }
  else if (pwm_right < -right_min_motor_speed && nextPwm > -right_min_motor_speed) {
    nextPwm += 2*right_min_motor_speed;
  }
  pwm_right = nextPwm;
  
  int max_speed = 400;
  pwm_left = max(-max_speed, min(max_speed, pwm_left));
  pwm_right = max(-max_speed, min(max_speed, pwm_right));
  float left_out = pwm_left;
  float right_out = pwm_right;
  md.setSpeeds(-left_out, -right_out);
  //printPIND();
  //printWorldCoords();
}
