
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

// Stores the total number of steps for both encoders
volatile long leftEncoder = 0;
volatile long rightEncoder = 0;
// Tracks the number of steps since the last world position update
volatile int leftSteps = 0;
volatile int rightSteps = 0;
const int stepsPerUpdate = 1;

// MOTION CONTROLLER CODE (JUST THE ONE LINE BELOW)
const float max_phi = (10.0f / 180.0f) * PI; // Max angle offset of robot (5 degrees currently)

const float botRadius = 4.125f;
const float wheelRadius = 1.39f;
const float stepDistance = 2.0f * wheelRadius * PI / 16.0f;

// World Coordinates
float worldX = 0.0f;  
float worldY = 0.0f;
float worldTheta = 0.0f;

// MOTION CONTROLLER CODE (BELOW)
// Previous World Coordinates
float prevWorldX = 0.0f;  
float prevWorldY = 0.0f;
float prevWorldTheta = 0.0f;
float prevWorldTime = millis();

// Current Velocity
float xVelocity = 0.0f;
float yVelocity = 0.0f;
float thetaVelocity = 0.0f;
// Goal location of the current goal we are trying to reach
float goalLocationX = 0.001;
float goalLocationY = 0;
float goalLocationTheta = atan2(goalLocationY, goalLocationX); //


float translate_velocity = 0.0f;
float turn_velocity = 0.0f;

float K = 4.5f;
float B = 0.13f;
// TODO FIND CORRECT K's and B's, MOTION CONTROLLER CODE 
float K_t = 1.0f;
float B_t = 0.1f;
float K_r = 0.0f;
float B_r = 0.0f;
float angle_ref = 0.02f;
float angular_rate_ref = 0.0f;

float pwm_left;
float pwm_right;

MPU6050 mpu;

int16_t ax;
int16_t ay;
int16_t az;
int16_t gx;
int16_t gy;
int16_t gz;
double pitch;

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
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  getPitch(&pitch, &ax, &az);
  //Serial.print(ax);
  //Serial.print(", ");
  //Serial.println(az);
  //angle and angular rate unit: radian
  double angle_err = pitch - angle_ref;               // angle_ref is center of gravity offset
  double angular_rate = PI*((double)gy)/180 - angular_rate_ref;     // converted to radians

  // MOTION CONTROLLER CODE (BELOW)
  // Translate Controller
  double distance_to_goal = pow((pow(goalLocationX - worldX, 2) + pow(goalLocationY - worldY, 2)), .5);
  double distance_direction = atan2(goalLocationX - worldX, goalLocationY - worldY) + worldTheta;
  if (distance_direction > PI || distance_direction < -PI)
    distance_to_goal *= -1;
  printWorldCoords();
  double velocity = (pow((pow(xVelocity, 2) + pow(yVelocity, 2)), .5));
  double velocity_direction = atan2(xVelocity, yVelocity) + worldTheta;
  if (!(velocity_direction > PI || velocity_direction < -PI) ^ (distance_direction > PI || distance_direction < -PI))
    velocity *= -1;
  Serial.print(distance_to_goal);
  Serial.print(", ");
  Serial.print(velocity);
  Serial.print(" :: ");
  double delta_phi = K_t * distance_to_goal - B_t * velocity;
  Serial.print(delta_phi);
  Serial.print(" -> ");
  if (delta_phi > max_phi) {
    delta_phi = max_phi;
  }
  if (delta_phi < -max_phi) {
    delta_phi = -max_phi;
  }
  angle_err += delta_phi;
  Serial.println(delta_phi);
  // Balance Controller
  float deltaPWM = -K * angle_err - B * angular_rate;
  // Rotate Controller
//  float deltaPWMR = K_r * (goalLocationTheta - worldTheta) - B_r * (thetaVelocity);
//  if (goalLocationTheta - worldTheta >= 0) {
//      pwm_left -= deltaPWMR;
//      pwm_right += deltaPWMR;
//  } else {
//      pwm_left += deltaPWMR;
//      pwm_right -= deltaPWMR;
//  }
  // MOTION CONTROLLER CODE (ABOVE)
  
  //float deltaPWM = -K*angle_err - B*angular_rate;
  //printAngleErrorAndSpeed(&angle_err, &angular_rate);
  //printDeltaPWMComponents(&deltaPWM, &K, &pitch, &angle_ref, &B, &gy);
  //printDeltaPWMEquation(&deltaPWM, &K, &pitch, &angle_ref, &B, &gy);
  //Serial.println(pwm_left);
  
  pwm_left += deltaPWM;
  pwm_right += deltaPWM;

  int max_speed = 300;
  if (pwm_left > max_speed)
    pwm_left = max_speed;
  if (pwm_right > max_speed)
    pwm_right = max_speed;            
  if (pwm_left < -max_speed)
    pwm_left = -max_speed;
  if (pwm_right < -max_speed)
    pwm_right = -max_speed;

  float leftOut = pwm_left;
  float rightOut = pwm_left;
//  if (leftOut < 0)
//    leftOut -= 20;
//  if (rightOut < 0)
//    rightOut -= 20;
//  if (leftOut > 0)
//    leftOut += 20;
//  if (rightOut > 0)
//    rightOut += 20;

  //Control motor
  md.setSpeeds(leftOut, rightOut);

  //Serial.println(analogRead(A0));
  //analog_encoder_two();
  //Serial.print(analogRead(A0));
  //Serial.print(", ");
//  Serial.println(analogRead(A1));
  //printPIND();
}

double pitchFlow = 1.0d;
double integratedPitch = 0.0d;
void getPitch(double *pitch, int16_t *ax, int16_t *az) {
  // Just using atan2 gives us an angle where a balanced position reads as either PI or -PI
  // Inverted so it's in the same direction as the gyro
  *pitch = -atan2(*ax, *az);
  *pitch = (1.0d - pitchFlow) * integratedPitch + pitchFlow * (*pitch);
  integratedPitch = *pitch;
}

// Initializing IR pins and interrupts
void init_IR()
{
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
  pinMode(IR_4, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  attachInterrupt(IR_INTERRUPT_1, encoder_one, CHANGE);
  attachInterrupt(IR_INTERRUPT_2, encoder_two, CHANGE);
  encoder_one(); // Initialize the encoders with the starting digital pins
  encoder_two();
}

// IR interrupts for the quadrature encoders
void encoder_one()
{
  // 0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111,
  // 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111
  static int8_t lookup_table[] = {0,1,0,-1, -1,0,1,0, 0,1,0,-1, -1,0,1,0};
  // Negative lookups correspond to the sensors moving with 2 leading
  static uint8_t enc_val = 0;
  
  enc_val = enc_val << 2;
  // PIND reads the first 8 digital pins (0-7)
  uint8_t pins = (PIND & 0b01000100) >> 2;
  pins = (pins & 0b1) | ((pins >> 3) & 0b10);
  enc_val = enc_val | pins;

  if (enc_val > 3) {
    leftEncoder -= lookup_table[enc_val & 0b1111];
    leftSteps -= lookup_table[enc_val & 0b1111]; 
  }
  else
    enc_val = enc_val | 0b10000;
  
  if (shouldUpdateCoords())
    updateCoords();

  
//  Serial.print("Left position: ");
//  Serial.print(leftEncoder);
//  Serial.print("    Left step: ");
//  Serial.println(leftSteps);
}

void encoder_two()
{
  // 0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111,
  // 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111
  static int8_t lookup_table[] = {0,1,0,-1, -1,0,1,0, 0,1,0,-1, -1,0,1,0};
  // Negative lookups correspond to the sensors moving with 2 leading
  static uint8_t enc_val = 0;
  
  enc_val = enc_val << 2;
  // PIND reads the first 8 digital pins (0-7)
  uint8_t pins = (PIND & 0b00101000) >> 3; // Interrupt on 3, regular on 5 (interrupts available only on 2,3)
  pins = (pins & 0b1) | ((pins >> 1) & 0b10);
  enc_val = enc_val | pins;

  if (enc_val > 3) {
    rightEncoder -= lookup_table[enc_val & 0b1111];
    rightSteps -= lookup_table[enc_val & 0b1111]; 
  }
  else
    enc_val = enc_val | 0b10000;
  
  if (shouldUpdateCoords())
    updateCoords();
    
  //Serial.print("Right position: ");
  //Serial.println(rightEncoder);
}

bool shouldUpdateCoords() {
  return abs(leftSteps) + abs(rightSteps) >= stepsPerUpdate;
}

void updateCoords() {
  float dsl = leftSteps * stepDistance;
  leftSteps = 0;
  float dsr = rightSteps * stepDistance;
  rightSteps = 0;
  float dsavg = (dsl + dsr) / 2;
  float dTheta = (dsl - dsavg)/botRadius;
//  Serial.print(leftEncoder);
//  Serial.print(" : ");
//  Serial.println(rightEncoder);
//  Serial.print("Left step: ");
//  Serial.print(leftSteps);
//  Serial.print("   dsl: ");
//  Serial.print(dsl);
//  Serial.print("   dsr: ");
//  Serial.print(dsr);
//  Serial.print("   dsavg: ");
//  Serial.print(dsavg);
//  Serial.print("   dthetha: ");
//  Serial.println(dTheta);
  //printWorldCoords();
  worldTheta += dTheta;
  while(worldTheta > PI)
    worldTheta -= 2*PI;
  while(worldTheta < -PI)
    worldTheta += 2*PI;
  worldX += dsavg * cos(worldTheta); // Or [worldTheta - (dTheta/2)]
  worldY += dsavg * sin(worldTheta); 
//  printWorldCoords();
  
   // MOTION CONTROLLER CODE
  double delta_t = (millis() - prevWorldTime) * 1000; // find time that has past and convert to seconds
  xVelocity = (worldX - prevWorldX) / delta_t;
  yVelocity = (worldY - prevWorldY) / delta_t;
  thetaVelocity = (worldTheta - prevWorldTheta) / delta_t;
  prevWorldX = worldX;
  prevWorldY = worldY;
  prevWorldTheta = worldTheta;
  prevWorldTime = millis();
}

void init_IO()
{
  // configure I/O
  pinMode(SPD_PUL_L, INPUT);
  pinMode(SPD_PUL_R, INPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L1, OUTPUT);
  pinMode(DIR_L2, OUTPUT);
  pinMode(DIR_R1, OUTPUT);
  pinMode(DIR_R2, OUTPUT);
}

void printDeltaPWMComponents(float *deltaPWM, float *K, double *pitch, float *angle_ref, float *B, int16_t *gy)
{
  Serial.print(*deltaPWM);
  Serial.print(" = ");
  Serial.print(-*K * (*pitch - *angle_ref));
  Serial.print(" + ");
  Serial.print(-*B * (PI/180) * (*gy));
  Serial.println();
}

void printDeltaPWMEquation(float *deltaPWM, float *K, double *pitch, float *angle_ref, float *B, int16_t *gy)
{
  Serial.print(*deltaPWM);
  Serial.print(" = ");
  Serial.print(*K);
  Serial.print(" * (");
  Serial.print(*pitch);
  Serial.print(" - ");
  Serial.print(*angle_ref);
  Serial.print(") - ");
  Serial.print(*B);
  Serial.print(" * (pi/180) * ");
  Serial.print(*gy);
  Serial.println();
}

void printAngleErrorAndSpeed(double *angle_err, double *angular_rate) {
  Serial.print("angle_err, angular_rate = ");
  Serial.print(*angle_err);
  Serial.print(", ");
  Serial.println(*angular_rate);
}

void printWorldCoords() {
  Serial.print("World coords (x, y, theta): (");
  Serial.print(worldX);
  Serial.print(", ");
  Serial.print(worldY);
  Serial.print(", ");
  Serial.print(worldTheta);
  Serial.println(")");
}

// For debugging
void printPIND()
{
  Serial.print((PIND & 0b10000000) >> 7);
  Serial.print((PIND & 0b01000000) >> 6);
  Serial.print((PIND & 0b00100000) >> 5);
  Serial.print((PIND & 0b00010000) >> 4);
  Serial.print((PIND & 0b00001000) >> 3);
  Serial.print((PIND & 0b00000100) >> 2);
  Serial.print((PIND & 0b00000010) >> 1);
  Serial.print((PIND & 0b00000001));
  Serial.println();
}
