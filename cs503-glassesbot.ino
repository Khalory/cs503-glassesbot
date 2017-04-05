
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
const int stepsPerUpdate = 3;

const float botRadius = 1.375f;
const float stepDistance = 2.0f * botRadius * PI / 64.0f;

// World Coordinates
float worldX = 0.0f;  
float worldY = 0.0f;
float worldTheta = 0.0f;

float translate_velocity = 0.0f;
float turn_velocity = 0.0f;

// 28, 22 work pretty well
// 45, 26 works best so far
// 55, 38 was decent but wobly
// 55, 25 is pretty pretty
float K = 4.5f;
float B = 0.13f;
float angle_ref = 0.02f;
float angular_rate_ref = 0.0f;
float wheel_rate_correction = 1.0f;//1.065f; // This gets multiplied by the wheel with more tilt so that we move straight

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

    // initialize device
    mpu.initialize();
    md.init();
    init_IR();

    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
}

void loop() {
  if (shouldUpdateCoords())
    updateCoords();
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  getPitch(&pitch, &ax, &az);
  //Serial.print(ax);
  //Serial.print(", ");
  //Serial.println(az);
  //angle and angular rate unit: radian
  double angle_err = pitch - angle_ref;               // angle_ref is center of gravity offset
  double angular_rate = PI*((double)gy)/180 - angular_rate_ref;     // converted to radians
  
  float deltaPWM = -K*angle_err - B*angular_rate;
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
  if (leftOut < 0)
    leftOut -= 20;
  if (rightOut < 0)
    rightOut -= 20;
  if (leftOut > 0)
    leftOut += 20;
  if (rightOut > 0)
    rightOut += 20;

  //Control motor
  //md.setSpeeds(leftOut, rightOut);

  //Serial.println(analogRead(A0));
  analog_encoder_two();
  //printPIND();
}

double pitchFlow = 1.0d;
double integratedPitch = 0.0d;
void getPitch(double *pitch, int16_t *ax, int16_t *az) {
  // Just using atan2 gives us an angle where a balanced position reads as either PI or -PI
  // Inverted so it's in the same direction as the gyro
  *pitch = -atan2(*ax, *az) - PI;
  if (*pitch < -PI)
    *pitch += 2*PI;
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
  //attachInterrupt(IR_INTERRUPT_2, encoder_two, CHANGE);
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
  uint8_t pins = (PIND & 0b01000100) >> 2; // Interrupt on 2, regular on 4 (interrupts available only on 2,3)
  pins = (pins & 0b1) | ((pins >> 3) & 0b10);
  enc_val = enc_val | pins;

  leftEncoder += lookup_table[enc_val & 0b1111];
  leftSteps += lookup_table[enc_val & 0b1111];
  //Serial.print("Left position: ");
  //Serial.println(leftEncoder);
}

const int top_thresh = 19;
const int bot_thresh = 15;
void analog_encoder_two()
{
  // 0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111,
  // 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111
  static int8_t lookup_table[] = {0,1,0,-1, -1,0,1,0, 0,1,0,-1, -1,0,1,0};
  static uint8_t enc_val = 0;
  int top = analogRead(A0);
  if (top > bot_thresh && top < top_thresh)
    return;
  int top_bin = top > top_thresh ? 1 : 0;
//  Serial.print(top_bin);
//  Serial.print(",, ");
//  Serial.println(enc_val);
  int bot = analogRead(A1);
//  Serial.print(top);
//  Serial.print(", ");
//  Serial.println(bot);
  if (top_bin == (enc_val & 1))
    return;

  int bot_bin = bot > (top_thresh+bot_thresh)/2 ? 1 : 0;

  uint8_t pins = top_bin | (bot_bin << 1);
  enc_val = (enc_val << 2) | pins;
  
  rightEncoder += lookup_table[enc_val & 0b1111];
  rightSteps += lookup_table[enc_val & 0b1111];
  Serial.print("Right position: ");
  Serial.println(rightEncoder);
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

  rightEncoder += lookup_table[enc_val & 0b1111];
  rightSteps += lookup_table[enc_val & 0b1111];
  Serial.print("Right position: ");
  Serial.println(rightEncoder);
}

bool shouldUpdateCoords() {
  return abs(leftSteps) + abs(rightSteps) >= stepsPerUpdate;
}

void updateCoords() {
  Serial.print(leftEncoder);
  Serial.print(" : ");
  Serial.println(rightEncoder);
  float dsl = leftSteps * stepDistance;
  leftSteps = 0; // Might have concurrency issues with the interrupt adding to these variables
  float dsr = rightSteps * stepDistance;
  rightSteps = 0;
  float dsavg = (dsl + dsr) / 2;
  float dTheta = (dsl - dsavg)/botRadius;

  worldTheta += dTheta;
  if (worldTheta > 2*PI)
    worldTheta -= 2*PI;
  if (worldTheta < 0)
    worldTheta += 2*PI;
  worldX += dsavg * cos(worldTheta); // Or [worldTheta - (dTheta/2)]
  worldY += dsavg * sin(worldTheta);
  printWorldCoords();
}

void init_IO()
{
  // configure I/O
  pinMode(SPD_PUL_L, INPUT);//
  pinMode(SPD_PUL_R, INPUT);//
  pinMode(PWM_L, OUTPUT);//
  pinMode(PWM_R, OUTPUT);//
  pinMode(DIR_L1, OUTPUT);//
  pinMode(DIR_L2, OUTPUT);
  pinMode(DIR_R1, OUTPUT);//
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

