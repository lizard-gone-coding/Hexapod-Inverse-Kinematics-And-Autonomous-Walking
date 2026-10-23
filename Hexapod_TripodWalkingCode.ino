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

// --- Balanced Low-Profile Settings ---
float groundY = 35.0;    
float groundZ = -45.0;   
float liftHeight = 10.0; // Subtle lift for stability
float strideX = 20.0;    // Shorter stride to prevent tipping

void setup() {
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < 3; j++) servos[i][j].attach(pins[i][j]);
  }
  stand(groundY, groundZ);
  delay(3000);
}

void loop() {
  moveTripod(true); 
  delay(50); 
  moveTripod(false);
  delay(50);
}

void moveTripod(bool groupA) {
  int steps = 30; 
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    
    // SWING: All swinging legs use the same math
    float swingX = -strideX + (2 * strideX * t); 
    float swingZ = groundZ + (liftHeight * sin(t * PI)); 
    
    // PUSH: All planted legs stay level
    float pushX = strideX - (2 * strideX * t);
    float pushZ = groundZ; 

    for (int leg = 0; leg < 6; leg++) {
      bool isGroupA = (leg == 0 || leg == 2 || leg == 4);
      if (groupA == isGroupA) {
        setLegIK(leg, swingX, groundY, swingZ, true);
      } else {
        setLegIK(leg, pushX, groundY, pushZ, false);
      }
    }
    delay(15); 
  }
}

void setLegIK(int leg, float x, float y, float z, bool isSwinging) {
  // --- Inverse Kinematics Core ---
  float theta1 = atan2(x, y) * 180.0 / PI;
  float L = sqrt(x * x + y * y) - L_COXA;
  float IK_H = sqrt(L * L + z * z);

  float cosT3 = (L_FEMUR * L_FEMUR + L_TIBIA * L_TIBIA - IK_H * IK_H) / (2 * L_FEMUR * L_TIBIA);
  float theta3 = acos(constrain(cosT3, -1, 1)) * 180.0 / PI;

  float alpha = atan2(z, L) * 180.0 / PI;
  float cosBeta = (L_FEMUR * L_FEMUR + IK_H * IK_H - L_TIBIA * L_TIBIA) / (2 * L_FEMUR * IK_H);
  float beta = acos(constrain(cosBeta, -1, 1)) * 180.0 / PI;
  float theta2 = alpha + beta;

  // Joint Bias: Keeping Tibia slightly extended to force Femur to lift
  if (isSwinging) {
    theta3 = theta3 * 0.75; 
  }

  int sTibia, sFemur, sHip;
  // Standardizing logic based on your wiring (LF, RM, LB, LM are Left-Style)
  bool isLeftLogic = (leg == 0 || leg == 2 || leg == 4 || leg == 5); 

  sHip = 90 + theta1; 
  if (isLeftLogic) {
    sFemur = 130 - theta2; 
    sTibia = 180 - theta3; 
  } else {
    sFemur = 60 + theta2;
    sTibia = 0 + theta3;
  }

  servos[leg][0].write(constrain(sTibia, 0, 180));
  servos[leg][1].write(constrain(sFemur, 0, 180));
  servos[leg][2].write(constrain(sHip, 0, 180));
}

void stand(float y, float z) {
  for(int i = 0; i < 6; i++) setLegIK(i, 0, y, z, false);
}