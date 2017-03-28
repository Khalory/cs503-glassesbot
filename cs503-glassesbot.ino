
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
float K = 12.0f;
float B = 0.5f;
float angle_ref = 0.02f;
float wheel_rate_correction = 1.0f;//1.065f; // This gets multiplied by the wheel with more tilt so that we move straight

int pwm_left;
int pwm_right;

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
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
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
    pinMode(2, INPUT);

    // supply your own gyro offsets here, scaled for min sensitivity
    //mpu.setXGyroOffset(220);
    //mpu.setYGyroOffset(76);
    //mpu.setZGyroOffset(-85);
    //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  getPitch(&pitch, &ax, &az);
  // angle and angular rate unit: radian
  float angle_err = pitch - angle_ref;               // angle_ref is center of gravity offset
  double angular_rate = PI*((double)gy)/180;     // converted to radians
  
  float deltaPWM = K*angle_err - B*angular_rate;
  printPitchAngleAndSpeed(&pitch, &angular_rate);
  //printDeltaPWMEquation(&deltaPWM, &K, &pitch, &angle_ref, &B, &gy);
  
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

  //Control motor
  md.setSpeeds(pwm_left/5, pwm_right*wheel_rate_correction/5);
}

const int xOffset = 0;
const int zOffset = 0;
void getPitch(double *pitch, int16_t *ax, int16_t *az) {
  *pitch = -atan2(*ax, *az) - PI;
  if (*pitch < -PI)
    *pitch += 2*PI;
}

// Initializing IR pins and interrupts
void init_IR()
{
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  attachInterrupt(0, encoder_one, CHANGE);
  attachInterrupt(1, encoder_two, CHANGE);
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
  uint8_t pins = (PIND & 0b00010100) >> 2; // Interrupt on 2, regular on 4 (interrupts available only on 2,3)
  pins = (pins & 0b1) | ((pins >> 1) & 0b10);
  enc_val = enc_val | pins;

  leftEncoder += lookup_table[enc_val & 0b1111];
  leftSteps += lookup_table[enc_val & 0b1111];
  //Serial.print("Left position: ");
  //Serial.println(leftEncoder);
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
  //Serial.print("Right position: ");
  //Serial.println(rightEncoder);
}

bool shouldUpdateCoords() {
  return leftSteps + rightSteps >= stepsPerUpdate;
}

void updateCoords() {
  float dsl = leftSteps * stepDistance;
  float dsr = rightSteps * stepDistance;
  float dsavg = (dsl + dsr) / 2;
  float dTheta = (dsl - dsavg)/botRadius;

  worldTheta += dTheta;
  if (worldTheta > 2*PI)
    worldTheta -= 2*PI;
  if (worldTheta < 0)
    worldTheta += 2*PI;
  worldX += dsavg * cos(worldTheta); // Or [worldTheta - (dTheta/2)]
  worldY += dsavg * sin(worldTheta);
}

void pwm_out(int l_val,int r_val)
{
  if (l_val<0)
  {
    digitalWrite(DIR_L1, HIGH);
    digitalWrite(DIR_L2, LOW);
    l_val=-l_val;
  }
  else
  {
    digitalWrite(DIR_L1, LOW);
    digitalWrite(DIR_L2, HIGH);
  }
  
  if (r_val<0)
  {
    digitalWrite(DIR_R1, HIGH);
    digitalWrite(DIR_R2, LOW);
    r_val=-r_val;
  }
  else
  {
    digitalWrite(DIR_R1, LOW);
    digitalWrite(DIR_R2, HIGH);
  }
  l_val=l_val+5;
  r_val=r_val+30;
  analogWrite(PWM_L, l_val>255? 255:l_val);
  analogWrite(PWM_R, r_val>255? 255:r_val);
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
  pinMode(A0, INPUT);digitalWrite(A0, HIGH);
  pinMode(A1, INPUT);digitalWrite(A1, HIGH);
  pinMode(A2, INPUT);digitalWrite(A2, HIGH);
  pinMode(A3, INPUT);digitalWrite(A3, HIGH);
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

void printPitchAngleAndSpeed(double *pitch, double *angular_rate) {
  Serial.print("pitch, angular_rate = ");
  Serial.print(*pitch);
  Serial.print(", ");
  Serial.println(*angular_rate);
}

