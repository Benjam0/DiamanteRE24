/* FourMotors.cpp
 * Autor: Pedro H. P. Silva
 * Date: 26/06/2023
 * Library criated to control four motors with encoder simultaneously
 * 
 */

#include <FourMotors.h>

FourMotors::FourMotors(DC_motor_controller * m1, DC_motor_controller* m2, DC_motor_controller* m3, DC_motor_controller*m4) {
  this->rightUp = m1; this->rightDown = m2;
  this->leftUp = m3; this->leftDown = m4;
}

void FourMotors::setGyreDegreesRatio(float rot, float ang) {
  this->rot_per_degree = rot / ang;
}

void FourMotors::together(float vel) {
  rightUp->walk(vel);
  rightDown->walk(vel);
  leftUp->walk(vel);
  leftDown->walk(vel);
}

void FourMotors::run(int pwm) {
  rightUp->run(pwm);
  rightDown->run(pwm);
  leftUp->run(pwm);
  leftDown->run(pwm);
}

void FourMotors::run(int pwm1, int pwm2, int pwm3, int pwm4) {
  rightUp->run(pwm1);
  rightDown->run(pwm2);
  leftUp->run(pwm3);
  leftDown->run(pwm4);
}

void FourMotors::together(float vel, float rot) {
  if (rot != 0) {
    resetMotors();
    while (rightUp->canRun() || rightDown->canRun() || leftUp->canRun() || leftDown->canRun()) {
      rightUp->gyrate(vel, rot);
      rightDown->gyrate(vel, rot);
      leftUp->gyrate(vel, rot);
      leftDown->gyrate(vel, rot);
    }
    resetMotors();

    unsigned long lastTime = millis();
    while (( millis() - lastTime) < 100 ) {
      rightUp->stop_both(0);
      rightDown->stop_both(0);
      leftUp->stop_both(0);
      leftDown->stop_both(0);
    }
    stop(200);
    resetMotors();
  } else {
    rightUp->walk(vel);
    rightDown->walk(vel);
    leftUp->walk(vel);
    leftDown->walk(vel);
  }
}

void FourMotors::together(float vel1, float rot1, float vel2, float rot2, float vel3, float rot3 , float vel4, float rot4) {
  resetMotors();
  while (rightUp->canRun() || rightDown->canRun() || leftUp->canRun() || leftDown->canRun()) {
    rightUp->gyrate(vel1, rot1);
    rightDown->gyrate(vel2, rot2);
    leftUp->gyrate(vel3, rot3);
    leftDown->gyrate(vel4, rot4);
  }
  stop(0);
}

void FourMotors::turnDegree(float vel, float degrees) {
  resetMotors();
  float rot = degrees * rot_per_degree;
  together(vel, rot, vel, rot, -vel, -rot, -vel, -rot);
}

void FourMotors::stop(unsigned int t) {
  if ( t < rightUp->getRefreshTime()) {
    resetMotors();
    rightUp->run(0);
    rightDown->run(0);
    leftUp->run(0);
    leftDown->run(0);
  }
  else {
    unsigned long lastT_local = millis();
    resetMotors();
    while ((millis() - lastT_local) < t) {
      rightUp->stop_both(0);
      rightDown->stop_both(0);
      leftUp->stop_both(0);
      leftDown->stop_both(0);;
    }
    rightUp->run(0);
    rightDown->run(0);
    leftUp->run(0);
    leftDown->run(0);
  }
}

void FourMotors::resetMotors() {
  rightUp->resetForGyrate();
  rightDown->resetForGyrate();
  leftUp->resetForGyrate();
  leftDown->resetForGyrate();
}

void FourMotors::reset() {
  resetMotors();
}
