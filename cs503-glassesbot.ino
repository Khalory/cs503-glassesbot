
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

float translate_velocity = 0.0f;
float turn_velocity = 0.0f;

// 28, 22 work pretty well
// 45, 26 works best so far
// 55, 38 was decent but wobly
// 55, 25 is pretty pretty
float K = 20.0f;
float B = 10.0f;
float angle_ref = 0.26f;
float wheel_rate_correction = 1.0f;//1.065f; // This gets multiplied by the wheel with more tilt so that we move straight

int pwm_left;
int pwm_right;

MPU6050 mpu;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t gyro[3];        // [x, y, z]            gyro vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Motor driver
DualMC33926MotorShield md;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

const int MPU_addr=0x68;  // I2C address of the MPU-6050
void setup() {
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Serial.println("l");
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("k");
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    init_IO();

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    md.init();
    pinMode(2, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  /*while (!mpuInterrupt) {
    
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    //Get sensor data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // angle and angular rate unit: radian
    float angle_err = ypr[1] - angle_ref;               // angle_ref is center of gravity offset
    double angular_rate = -((double)gyro[1]/131.0);     // converted to radians
    //Serial.println(ypr[1]);
    //Serial.print(", ");
    //Serial.print(angle_err);
    //Serial.println();
    //Serial.print("Error, rate: ");
    //Serial.print(angle_err);
    //Serial.print(", ");
    //Serial.println(angle_ref);

    //displayYPR();
    
    float deltaPWM = K*angle_err - B*angular_rate + translate_velocity;
    //Serial.print("K: ");
    //Serial.println(K*angle_err);
    //Serial.print("B: ");
    //Serial.println(-B*angular_rate);
    //Serial.print("Delta PWM: ");
    //Serial.println(deltaPWM);
    pwm_left += deltaPWM + turn_velocity;
    pwm_right += deltaPWM - turn_velocity;

    int max_speed = 300;
    if (pwm_left > max_speed)
      pwm_left = max_speed;
    if (pwm_right > max_speed)
      pwm_right = max_speed;
    if (pwm_left < -max_speed)
      pwm_left = -max_speed;
    if (pwm_right < -max_speed)
      pwm_right = -max_speed;

    //Serial.print("Motor Power: ");
    //Serial.print(pwm_left);
    //Serial.print("\t");
    //Serial.println(pwm_right);

    // Control motor
    //pwm_out(pwm_left, pwm_right);
    md.setSpeeds(pwm_left, pwm_right*wheel_rate_correction);
  }*/
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  //Get sensor data
  quaternion(&q, fifoBuffer);
  mpu.dmpGetGyro(gyro, fifoBuffer);
  getGravity(&gravity, &q);
  YPR(ypr, &q, &gravity);
  Serial.println("--");
  Serial.println(ypr[1]);
  Serial.println(gyro[1]);
  // angle and angular rate unit: radian
  float angle_err = ypr[1] - angle_ref;               // angle_ref is center of gravity offset
  double angular_rate = -((double)gyro[1]/131.0);     // converted to radians
    
  float deltaPWM = K*angle_err - B*angular_rate + translate_velocity;
  //Serial.print("K: ");
  //Serial.println(K*angle_err);
  //Serial.print("B: ");
  //Serial.println(-B*angular_rate);
  //Serial.print("Delta PWM: ");
  //Serial.println(deltaPWM);
  pwm_left += deltaPWM + turn_velocity;
  pwm_right += deltaPWM - turn_velocity;

  int max_speed = 300;
  if (pwm_left > max_speed)
    pwm_left = max_speed;
  if (pwm_right > max_speed)
    pwm_right = max_speed;
  if (pwm_left < -max_speed)
    pwm_left = -max_speed;
  if (pwm_right < -max_speed)
    pwm_right = -max_speed;

  //Serial.print("Motor Power: ");
  //Serial.print(pwm_left);
  //Serial.print("\t");
  //Serial.println(pwm_right);

  // Control motor
  //pwm_out(pwm_left, pwm_right);
  md.setSpeeds(pwm_left, pwm_right*wheel_rate_correction);
}

void displayYPR() {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
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

void getData(uint8_t *data) {
  /*Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 42, true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)*/
  readBytes(MPU_addr, 0x74, 42, data);
}

void readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.endTransmission();
  Wire.beginTransmission(devAddr);
  Wire.requestFrom(devAddr, (uint8_t)min(length, BUFFER_LENGTH));

    int8_t count = 0;
  for (; Wire.available(); count++) {
      data[count] = Wire.read();
      #ifdef I2CDEV_SERIAL_DEBUG
          Serial.print(data[count], HEX);
          if (count + 1 < length) Serial.print(" ");
      #endif
  }

  Wire.endTransmission();
}

uint8_t *dmpPacketBuffer;
void quaternion(Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    if (packet == 0) packet = dmpPacketBuffer;
    qI[0] = ((packet[0] << 8) | packet[1]);
    qI[1] = ((packet[4] << 8) | packet[5]);
    qI[2] = ((packet[8] << 8) | packet[9]);
    qI[3] = ((packet[12] << 8) | packet[13]);
    
    q -> w = (float)qI[0] / 16384.0f;
    q -> x = (float)qI[1] / 16384.0f;
    q -> y = (float)qI[2] / 16384.0f;
    q -> z = (float)qI[3] / 16384.0f;
}

void getGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
}

void YPR(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
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
