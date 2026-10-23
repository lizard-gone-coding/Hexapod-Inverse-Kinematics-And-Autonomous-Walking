#include <Servo.h>
#include <math.h>

// --- Leg Dimensions ---
const float L_COXA = 25.0;
const float L_FEMUR = 50.0;
const float L_TIBIA = 70.0;

Servo servos[6][3]; 
const int pins[6][3] = {
  {9, 8, 7}, {6, 5, 4}, {10, 3, 2}, {25, 27, 29}, {33, 35, 37}, {50, 52, 48}
};

// --- Ultra-Tucked Ripple Settings ---
float groundY = 32.0;    // Absolute minimum before hitting chassis
float groundZ = -45.0;   
float liftHeight = 15.0; 
float strideX = 20.0;    
int stepDelay = 12;      

void setup() {
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < 3; j++) {
      servos[i][j].attach(pins[i][j]);
      // Set a neutral safe start
      servos[i][j].write(90);
    }
  }
  delay(1000);
  stand(groundY, groundZ);
  delay(2000);
}

void loop() {
  int rippleOrder[] = {0, 2, 4, 1, 5, 3};

  for (int i = 0; i < 6; i++) {
    moveSingleLeg(rippleOrder[i]);
  }
}

/**
 * Moves one leg while "Refreshing" the position of the other 5.
 * This acts as a software "reset" to ensure they stay at groundZ.
 */
void moveSingleLeg(int legIndex) {
  int steps = 20; 
  
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float swingX = -strideX + (2 * strideX * t); 
    float swingZ = groundZ + (liftHeight * sin(t * PI)); 

    for (int l = 0; l < 6; l++) {
      if (l == legIndex) {
        // ACTIVE LEG: Swinging
        setLegIK(l, swingX, groundY, swingZ, true);
      } else {
        // IDLE LEGS: Constantly re-sending the "Stand" command.
        // This fights back-driving by keeping the PWM signal fresh.
        setLegIK(l, 0, groundY, groundZ, false);
      }
    }
    delay(stepDelay);
  }
}

void setLegIK(int leg, float x, float y, float z, bool isSwinging) {
  float theta1 = atan2(x, y) * 180.0 / PI;
  float L = sqrt(x * x + y * y) - L_COXA;
  float IK_H = sqrt(L * L + z * z);

  float cosT3 = (L_FEMUR * L_FEMUR + L_TIBIA * L_TIBIA - IK_H * IK_H) / (2 * L_FEMUR * L_TIBIA);
  float theta3 = acos(constrain(cosT3, -1, 1)) * 180.0 / PI;

  float alpha = atan2(z, L) * 180.0 / PI;
  float cosBeta = (L_FEMUR * L_FEMUR + IK_H * IK_H - L_TIBIA * L_TIBIA) / (2 * L_FEMUR * IK_H);
  float beta = acos(constrain(cosBeta, -1, 1)) * 180.0 / PI;
  float theta2 = alpha + beta;

  // Favoring the Upper Knee (Femur)
  if (isSwinging) {
    theta3 = theta3 * 0.70; 
  }

  int sTibia, sFemur, sHip;
  bool isLeftLogic = (leg == 0 || leg == 2 || leg == 4 || leg == 5); 

  sHip = 90 + theta1; 
  if (isLeftLogic) {
    sFemur = 130 - theta2; 
    sTibia = 180 - theta3; 
  } else {
    sFemur = 60 + theta2;
    sTibia = 0 + theta3;
  }

  // REFRESH COMMAND: Forcing the update to the servo hardware
  servos[leg][0].write(constrain(sTibia, 0, 180));
  servos[leg][1].write(constrain(sFemur, 0, 180));
  servos[leg][2].write(constrain(sHip, 0, 180));
}

void stand(float y, float z) {
  for(int i = 0; i < 6; i++) setLegIK(i, 0, y, z, false);
}