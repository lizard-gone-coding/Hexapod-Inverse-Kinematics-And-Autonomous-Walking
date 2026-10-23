#include <Servo.h>

// Right Front Leg
Servo servoRF1; // Lower knee joint
Servo servoRF2; // Upper knee joint
Servo servoRF3; // Hip joint

// Right Mid Leg
Servo servoRM1; // Lower knee joint
Servo servoRM2; // Upper knee joint
Servo servoRM3; // Hip joint

// Right Back Leg
Servo servoRB1; // Lower knee joint
Servo servoRB2; // Upper knee joint
Servo servoRB3; // Hip joint

// Left Front Leg
Servo servoLF1;  // Lower knee joint
Servo servoLF2;  // Upper knee joint
Servo servoLF3;  // Hip joint

// Left Mid Leg
Servo servoLM1;  // Lower knee joint
Servo servoLM2;  // Upper knee joint
Servo servoLM3;  // Hip joint

// Left Back Leg
Servo servoLB1;  // Lower knee joint
Servo servoLB2;  // Upper knee joint
Servo servoLB3;  // Hip joint

void setup() {
  //check
  servoLF1.attach(9);
  servoLF2.attach(8); //reversed
  servoLF3.attach(7);

  servoLF1.write(180);
  servoLF2.write(130);
  servoLF3.write(90);
//--------------------
//check
  servoRF1.attach(6); //reversed
  servoRF2.attach(5);
  servoRF3.attach(4);

  servoRF1.write(0);
  servoRF2.write(60);
  servoRF3.write(90);
//--------------------
//check
  servoRM1.attach(10);
  servoRM2.attach(3);  //reversed
  servoRM3.attach(2);

  servoRM1.write(180);
  servoRM2.write(130);
  servoRM3.write(90);
//--------------------
//check
  servoRB1.attach(25);  //reversed
  servoRB2.attach(27);
  servoRB3.attach(29);

  servoRB1.write(0);
  servoRB2.write(60);
  servoRB3.write(90);
//--------------------
// check
  servoLB1.attach(33);
  servoLB2.attach(35); //reversed
  servoLB3.attach(37);

  servoLB1.write(180);
  servoLB2.write(130);
  servoLB3.write(90);
//--------------------
// check
  servoLM1.attach(50);
  servoLM2.attach(52); //reversed
  servoLM3.attach(48);

  servoLM1.write(180);
  servoLM2.write(130);
  servoLM3.write(90);

}

void loop() {
  //Left Focus Forward Step
  servoLF2.write(160); //pick up left focus tripod
  servoLB2.write(160);
  servoRM2.write(160);
  delay(200);
  servoRF3.write(90); //Right focus move back side
  servoRB3.write(90);
  servoLM3.write(90);
  
  servoLF3.write(45); //Left focus move side
  servoLB3.write(45);
  servoRM3.write(45);
  delay(200);
  servoLF2.write(120); //put down left focus tripod
  servoLB2.write(110);
  servoRM2.write(120);
  delay(100);
  servoLF2.write(100); //correction so as not to slip down
  servoLB2.write(90);
  servoRM2.write(100);
  delay(100);
  servoLF2.write(120);
  servoLB2.write(110);
  servoRM2.write(120);
  delay(200);
  
  servoRF2.write(30); //pick up right focus tripod
  servoRB2.write(30);
  servoLM2.write(160);
  delay(200);
  servoLF3.write(90); //Left Focus move back side
  servoLB3.write(90);
  servoRM3.write(90);
  
  servoRF3.write(135); //Right Focus move side
  servoRB3.write(135);
  servoLM3.write(135);
  delay(200);
  servoRF2.write(70); //Put down Right Focus tripod
  servoRB2.write(80);
  servoLM2.write(120);
  delay(100);
  servoRF2.write(80); //correction so as not to slip down
  servoRB2.write(90);
  servoLM2.write(110);
  delay(100);
  servoRF2.write(70);
  servoRB2.write(80);
  servoLM2.write(120);
  delay(200);
  
  
  //Right Focus Forward Step
  /*servoRF2.write(30);
  servoRB2.write(30);
  servoLM2.write(160);
  delay(1000);
  servoRF3.write(135);
  servoRB3.write(135);
  servoLM3.write(135);
  delay(1000);
  servoRF2.write(60);
  servoRB2.write(60);
  servoLM2.write(130);
  delay(1000);
  servoRF3.write(90);
  servoRB3.write(90);
  servoLM3.write(90);
  delay(1000);*/
 
}
