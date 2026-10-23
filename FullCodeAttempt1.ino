#include <Servo.h>
#include <math.h>

// --- SLAM / Mapping Constants ---
const int MAP_SIZE = 20;
const float CELL_SIZE = 3.6; 
int internalMap[MAP_SIZE][MAP_SIZE];

// --- Robot Pose & Tracking ---
float robotX = 10.0, robotY = 10.0; 
float robotAngle = 0;              
float distanceTraveled = 0;
const float STEP_DIST = 2.2; 

// --- Servo Setup ---
Servo servos[6][3]; 
const int pins[6][3] = {
  {9, 8, 7},   // Leg 0: LF
  {6, 5, 4},   // Leg 1: RF
  {10, 3, 2},  // Leg 2: RM
  {25, 27, 29},// Leg 3: RB
  {33, 35, 37},// Leg 4: LB
  {50, 52, 48} // Leg 5: LM
};

const int trigPin = 11;
const int echoPin = 12;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < 3; j++) servos[i][j].attach(pins[i][j]);
  }
  
  for(int x=0; x<MAP_SIZE; x++) {
    for(int y=0; y<MAP_SIZE; y++) internalMap[x][y] = 0;
  }
  standReady();
  delay(3000); 
}

void executeMappedIK(int leg, float x, float z) {
  int targetHip, targetFemur, targetTibia;
  float hipOffset = x; 
  
  if (leg == 0 || leg == 1 || leg == 2 || leg == 4) {
    targetHip = 90 - hipOffset; 
  } else { 
    targetHip = 90 + hipOffset; 
  }

  bool isTypeA = (leg == 0 || leg == 2 || leg == 4 || leg == 5); 

  if (isTypeA) {
    targetTibia = 180; 
    if (z > 0)        targetFemur = 160; 
    else if (z == 0)  targetFemur = 130; 
    else if (z == -75) targetFemur = 120; 
    else if (z == -85) targetFemur = 100; 
    else              targetFemur = 130;
  } else {
    targetTibia = 0;   
    if (z > 0)        targetFemur = 20;  
    else if (z == 0)  targetFemur = 60;  
    else if (z == -75) targetFemur = 70;  
    else if (z == -85) targetFemur = 90;  
    else              targetFemur = 60;
  }

  servos[leg][2].write(targetHip);
  servos[leg][1].write(targetFemur);
  servos[leg][0].write(targetTibia);
}

void walkForwardMapped() {
  // TRIPOD A
  executeMappedIK(0, 0, 15); executeMappedIK(4, 0, 15); executeMappedIK(2, 0, 15);
  delay(200);
  executeMappedIK(0, 30, 15); executeMappedIK(4, 30, 15); executeMappedIK(2, 30, 15);
  executeMappedIK(1, 0, -75); executeMappedIK(3, 0, -75); executeMappedIK(5, 0, -75);
  delay(200);
  executeMappedIK(0, 30, -75); executeMappedIK(4, 30, -75); executeMappedIK(2, 30, -75);
  delay(200);

  // TRIPOD B
  executeMappedIK(1, 0, 15); executeMappedIK(3, 0, 15); executeMappedIK(5, 0, 15);
  delay(200);
  executeMappedIK(1, 30, 15); executeMappedIK(3, 30, 15); executeMappedIK(5, 30, 15);
  executeMappedIK(0, 0, -75); executeMappedIK(4, 0, -75); executeMappedIK(2, 0, -75);
  delay(200);
  executeMappedIK(1, 30, -75); executeMappedIK(3, 30, -75); executeMappedIK(5, 30, -75);
  delay(200);
}

void turnSequence(bool right) {
  float turnDir = right ? 30 : -30;
  
  // 1. Lift Tripod A (LF, LB, RM)
  executeMappedIK(0, 0, 15); executeMappedIK(4, 0, 15); executeMappedIK(2, 0, 15);
  delay(200);
  
  // 2. Rotate Tripod A while Tripod B stays grounded
  executeMappedIK(0, turnDir, 15); executeMappedIK(4, turnDir, 15); executeMappedIK(2, turnDir, 15);
  delay(200);
  
  // 3. Plant Tripod A
  executeMappedIK(0, turnDir, -75); executeMappedIK(4, turnDir, -75); executeMappedIK(2, turnDir, -75);
  delay(200);

  // 4. Lift Tripod B (RF, RB, LM) to reset for next movement
  executeMappedIK(1, 0, 15); executeMappedIK(3, 0, 15); executeMappedIK(5, 0, 15);
  delay(200);
  
  // 5. Rotate Tripod B in the opposite relative direction to "reset" hip positions
  executeMappedIK(1, -turnDir, 15); executeMappedIK(3, -turnDir, 15); executeMappedIK(5, -turnDir, 15);
  delay(200);
  
  // 6. Plant Tripod B and Return to Neutral
  standReady();
}

void loop() {
  long distance = getDistance();
  if (distanceTraveled >= 24.0) { performSpinScan(); distanceTraveled = 0; }
  if (distance > 0 && distance < 10) {
    turnSequence(true); 
    updateOdometry(0, 30);
  } else {
    walkForwardMapped();
    updateOdometry(STEP_DIST, 0);
    updateMap(distance);
    distanceTraveled += STEP_DIST;
  }
}

void performSpinScan() {
  for(int i=0; i<6; i++) { // Reduced count for faster mapping
    turnSequence(true);
    updateMap(getDistance());
    delay(100);
  }
}

long getDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long result = (duration / 2) / 74;
  return (result <= 0) ? 999 : result;
}

void standReady() { for(int i=0; i<6; i++) executeMappedIK(i, 0, 0); delay(500); }

void updateOdometry(float d, float a) {
  robotAngle += a;
  float rad = robotAngle * PI / 180.0;
  robotX += (cos(rad) * d / CELL_SIZE);
  robotY += (sin(rad) * d / CELL_SIZE);
}

void updateMap(long dist) {
  if (dist > 30 || dist <= 0) return;
  float rad = robotAngle * PI / 180.0;
  int obsX = (int)(robotX + (cos(rad) * dist / CELL_SIZE));
  int obsY = (int)(robotY + (sin(rad) * dist / CELL_SIZE));
  if (obsX >= 0 && obsX < MAP_SIZE && obsY >= 0 && obsY < MAP_SIZE) internalMap[obsX][obsY]++;
}