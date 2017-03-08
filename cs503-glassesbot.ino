
float translate_velocity = 0.0f;
float turn_velocity = 0.0f;

float K = 60.0f;
float B = 60.0f;
float angle_ref = 0.0f;

void setup() {

}

void loop() {

  // Read gyro
  
  float deltaPWM = K*(angle - angle_ref) + B*(angle_rate);
  static float PWMout += deltaPWM;
  PWMout += translate_velocity;
  static float PWMout_left -= turn_velocity;
  static float PWMout_right += turn_velocity;

  // Control motor
}
