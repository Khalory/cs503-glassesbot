void updateVelocity() {
  for (int i = velocityUpdateCount-1; i > 0; i--) {
    prevWorldXs[i] = prevWorldXs[i-1];
    prevWorldYs[i] = prevWorldYs[i-1];
    prevWorldThetas[i] = prevWorldThetas[i-1];
    prevWorldTimes[i] = prevWorldTimes[i-1];
  }
  prevWorldXs[0] = worldX;
  prevWorldYs[0] = worldY;
  prevWorldThetas[0] = worldTheta;
  prevWorldTimes[0] = millis();
  velocityUpdateTimer = prevWorldTimes[0];
  
  double delta_t = (millis() - prevWorldTimes[velocityUpdateCount-1]) * 0.001; // find time that has past and convert to seconds
  xVelocity = (worldX - prevWorldXs[velocityUpdateCount-1]) / delta_t;
  yVelocity = (worldY - prevWorldYs[velocityUpdateCount-1]) / delta_t;
  thetaVelocity = (worldTheta - prevWorldThetas[velocityUpdateCount-1]) / delta_t;
}
void getPitch(double *pitch, int16_t *ax, int16_t *az) {
  // Just using atan2 gives us an angle where a balanced position reads as either PI or -PI
  // Inverted so it's in the same direction as the gyro
  *pitch = -atan2(*ax, *az) - PI;
  if (*pitch < -PI)
    *pitch += 2*PI;
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
bool isFirstLeft = true;
// IR interrupts for the quadrature encoders
void encoder_one()
{
  // 0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111,
  // 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111
  static int8_t lookup_table[] = {0,1,0,-1, -1,0,1,0, 0,1,0,-1, -1,0,1,0};
  // Negative lookups correspond to the sensors moving with 2 leading
  static uint8_t enc_val = 0;

  if (!isFirstLeft && (PIND & 0b100) >> 2 == (enc_val & 1)) {
    return;
  }
  
  enc_val = enc_val << 2;
  // PIND reads the first 8 digital pins (0-7)
  uint8_t pins = (PIND & 0b01000100) >> 2;
  pins = (pins & 0b1) | ((pins >> 3) & 0b10);
  enc_val = enc_val | pins;
  if (!isFirstLeft) {
    leftEncoder -= lookup_table[enc_val & 0b1111];
    leftSteps -= lookup_table[enc_val & 0b1111]; 
  }
  else
    isFirstLeft = false;
//  print8Bit(enc_val);
  
//  Serial.print("Left position: ");
//  Serial.println(leftEncoder);
}
bool isFirstRight = true;
void encoder_two()
{
  // 0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111,
  // 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111
  static int8_t lookup_table[] = {0,1,0,-1, -1,0,1,0, 0,1,0,-1, -1,0,1,0};
  // Negative lookups correspond to the sensors moving with 2 leading
  static uint8_t enc_val = 0;

  if (!isFirstRight && (PIND & 0b1000) >> 3 == (enc_val & 1)) {
    return;
  }
  
  enc_val = enc_val << 2;
  // PIND reads the first 8 digital pins (0-7)
  uint8_t pins = (PIND & 0b00101000) >> 3; // Interrupt on 3, regular on 5 (interrupts available only on 2,3)
  pins = (pins & 0b1) | ((pins >> 1) & 0b10);
  enc_val = enc_val | pins;
  if (!isFirstRight) {
    rightEncoder += lookup_table[enc_val & 0b1111];
    rightSteps += lookup_table[enc_val & 0b1111]; 
  }
  else
    isFirstRight = false;
  //print8Bit(enc_val);
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
  float totalDsavg = (stepDistance*(leftEncoder + rightEncoder)) / 2;
//  float dTheta = (dsl - dsavg)/botRadius;
//  worldTheta += dTheta;
  worldTheta = (stepDistance*leftEncoder - totalDsavg) / botRadius;
  //Serial.println(worldTheta);
  while(worldTheta > PI)
    worldTheta -= 2*PI;
  while(worldTheta <= -PI)
    worldTheta += 2*PI;
  worldX += dsavg * cos(worldTheta); // Or [worldTheta - (dTheta/2)]
  worldY += dsavg * sin(worldTheta);
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
void print8Bit(uint8_t val)
{
  Serial.print((val & 0b10000000) >> 7);
  Serial.print((val & 0b01000000) >> 6);
  Serial.print((val & 0b00100000) >> 5);
  Serial.print((val & 0b00010000) >> 4);
  Serial.print((val & 0b00001000) >> 3);
  Serial.print((val & 0b00000100) >> 2);
  Serial.print((val & 0b00000010) >> 1);
  Serial.print((val & 0b00000001));
  Serial.println();
}
