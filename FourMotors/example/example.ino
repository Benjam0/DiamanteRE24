//============LIBRARIES===========================
#include <DC_motor_controller.h>
#include <FourMotors.h>
//============CONSTANTS===========================

//============VARIABLES===========================

//============OBJECTS=============================

//============FUNCTIONS_DECLARATIONS==============


DC_motor_controller rightUp, rightDown , leftUp, leftDown;
FourMotors motors = FourMotors(&rightUp, &rightDown, &leftUp, &leftDown);


void intRU() {
  rightUp.isr();
}

void intRD() {
  rightDown.isr();
}

void intLU() {
  leftUp.isr();
}

void intLD() {
  leftDown.isr();
}

void setup() {
  rightUp.hBridge(6, 7);
  rightUp.setEncoderPin(15, 14);
  rightUp.setPPR(11);
  rightUp.setRR(30);
  rightUp.setPIDconstants(1.2, 0.2, 0.015);
  rightUp.setPins();
  attachInterrupt(15, intRU, FALLING);

  rightDown.hBridge(8, 9);
  rightDown.setEncoderPin(12, 13);
  rightDown.setPPR(11);
  rightDown.setRR(30);
  rightDown.setPIDconstants(1.2, 0.2, 0.015);
  rightDown.setPins();
  attachInterrupt(12, intRD, FALLING);


  leftUp.hBridge(2, 3);
  leftUp.setEncoderPin(1, 0);
  leftUp.setPPR(11);
  leftUp.setRR(30);
  leftUp.setPIDconstants(1.2, 0.2, 0.015);
  leftUp.setPins();
  attachInterrupt(1, intLU, FALLING);

  leftDown.hBridge(4, 5);
  leftDown.setEncoderPin(11, 10);
  leftDown.setPPR(11);
  leftDown.setRR(30);
  leftDown.setPIDconstants(1.2, 0.1, 0.02);
  leftDown.setPins();
  attachInterrupt(11, intLD, FALLING);

  motors.setGyreDegreesRatio(1.25, 90);

  

}

void loop() {

}

//============FUNCTIONS===========================
