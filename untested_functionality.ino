bool isMovingForward() {
  double direction_of_velocity = atan2(yVelocity, xVelocity);
  double theta_difference = clampThetaDifference(worldTheta - direction_of_velocity);
  
  if (abs(theta_difference) > PI/2) {
    return false;
  }
  return true;
}

double clampThetaDifference(double theta_difference) {
   if (theta_difference > PI) {
    return -2 * PI + theta_difference;
  } 
  else if (theta_difference < -PI) {
    return 2 * PI + theta_difference;
  }
  return theta_difference;
}

void resetChecks() {
  checkTurnLeft = 0;
  checkTurnRight = 0;
  checkStraight = 0;
}

void handleRightTurn(double inchesRight) {
  // If we are starting our right turn, first move forward
  if(turnRightState == 0) {
    turnRightState = 1;
    goalLocationX = worldX - 3*cos(worldTheta); // Move 5 inches forward after we run off the wall
    goalLocationY = worldY - 3*sin(worldTheta);
  }
  // Check if we've moved forward enough and then start turning right if we have
  if(turnRightState == 1) {
    float distance_to_goal = (pow(goalLocationX - worldX, 2) + pow(goalLocationY - worldY, 2));
    if (distance_to_goal < 2.0f) {
      turnRightState = 2;
      goalLocationTheta = worldTheta + PI/2;
      if (goalLocationTheta > PI)
        goalLocationTheta -= 2*PI;
    }
  }
    // Once we've turned close to the correct angle, then go straight again
  if(turnRightState == 2) {
    if (abs(clampThetaDifference(worldTheta - goalLocationTheta)) < PI * 0.1) {
      turnRightState = 3;
    }
  }
  // Move straight until we see a wall on our right, then change to a straight state
  if(turnRightState == 3) {
    if(inchesRight < MaxRightDist) {
      if (++checkStraight == checkStraightMax) 
      {
        turnLeft = false;
        turnRight = false;
        turnRightState = 0;
      }
    }
    else
      checkStraight = 0;
  }
}

void getValuesFromPing() {
  double inchesRight, inchesFront;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  // Calculating for Right Ping
  inchesRight = pingInches(pingPinRight);
  
  // Calculating for Front Ping
  inchesFront = pingInches(pingPinFront);
//    Serial.print(inchesRight);
//     Serial.print(": ");
//      Serial.println(inchesFront);
    
 
  for (int i = 1; i < NUM_PINGS; i++)
    pings[i-1] = pings[i];
  pings[NUM_PINGS-1] = inchesRight;
  avgPingChange = avgPingFlow * avgPingChange + (1 - avgPingFlow)*(inchesRight - pings[0]);
  
  pings[NUM_PINGS-1] = inchesRight;

  if(turnRight) {
    handleRightTurn(inchesRight);
    return;
  }
  
  // We are inside the appropriate range of the right ping sensor
  if(inchesRight < MaxRightDist) 
  {
    // There is a wall in front of us so turn left
    // TODO should calibrate MinFrontDist
    if(inchesFront < MinFrontDist) 
    {
      if(!turnLeft && ++checkTurnLeft == checkTurnLeftMax) 
      {
        checkTurnLeft = 0;
        turnLeft = true;
        turnRight = false;
        goalLocationTheta = worldTheta - PI/2;
        if(goalLocationTheta < -PI)
          goalLocationTheta += 2*PI;
        //Serial.println("Turn Left");
      }
      checkStraight = 0;
      checkTurnRight = 0;
    } 
    else 
    {
      // If we were turning, we're near our goal theta, and there's a wall to our right, then move straight
      if ((turnLeft || turnRight) && (abs(clampThetaDifference(worldTheta - goalLocationTheta)) > PI * 0.2) && ++checkStraight == checkStraightMax) 
      {
        checkStraight = 0;
        turnLeft = false;
        turnRight = false;
      }
      checkTurnLeft = 0;
      checkTurnRight = 0;
      //resetChecks();
      //Serial.println("Straight");
    }
  } 
  else {
    if(!turnRight && ++checkTurnRight == checkTurnRightMax) {
      checkTurnRight = 0;
      turnRight = true;
      turnLeft = false;
      handleRightTurn(inchesRight);
    }
    checkTurnLeft = 0;
    checkStraight = 0;
    
  }
}

float generateGoals(float *xs, float *ys) {
  float radius = 20.0f;
  int counter = 0;
  float step = curveAngle / (float)numPoints;
  for (float t = curveAngle; t >= -0.001 && counter < numPoints; t -= step) {
    xs[counter] = radius*sin(t);
    ys[counter] = radius*(1 + cos(t));
    counter++;
  }
}

int currentGoal = 0;
const float stickLength = 6.0f;
void updateGoal()
{
  float stickX = cos(worldTheta);
  float stickY = sin(worldTheta);
  // If we're close to the current goal
  float closeness = pow(goalXs[currentGoal] - worldX, 2) + pow(goalYs[currentGoal] - worldY, 2);
  Serial.print(currentGoal);
  Serial.print(": ");
  Serial.println(closeness);
  if (closeness < stickLength * stickLength * 2) {
    currentGoal++;
  }
  float dist;
  float minDist = 10000.0f;
  int minIndex = currentGoal;
//  printWorldCoords();
  for (int i = currentGoal; i < currentGoal + 5 && i < numPoints; i++) {
    dist = pow(goalXs[i] - stickLength*stickX - worldX, 2) + pow(goalYs[i] - stickLength*stickY - worldY, 2);
    if (dist < minDist) {
      minDist = dist;
      minIndex = i;
    }
  }
  
  if (currentGoal != minIndex) {
    goalLocationX = goalXs[minIndex];
    goalLocationY = goalYs[minIndex];
    currentGoal = minIndex;
    Serial.println(currentGoal);
  }
  
  float forwardX = goalLocationX - worldX;
  float forwardY = goalLocationY - worldY;
  float forwardNorm = forwardX*forwardX + forwardY*forwardY;
  forwardX /= forwardNorm;
  forwardY /= forwardNorm;
  float distance = stickX * forwardX + stickY * forwardY; // Dot product
  goalLocationTheta = atan2(forwardX, forwardY);
  stickX *= distance;
  stickY *= distance;
}

// Calculate the inches from wall
double pingInches(int pin) {
  long duration;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);
  // convert the time into a distance
  return microsecondsToInches(duration);
}

// Get inches from PING sensor values
double microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return (double)microseconds / 73.746d / 2.0d;
}
