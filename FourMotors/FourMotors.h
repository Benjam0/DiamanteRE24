/* FourMotors.h
 * Autor: Pedro H. P. Silva
 * Date: 26/06/2023
 * Library criated to control four motors with encoder simultaneously
 * 
 */

#ifndef FourMotors_h
#define FourMotors_h

#include "Arduino.h"
#include <DC_motor_controller.h>

 class FourMotors {
  public:

    DC_motor_controller* rightUp = new DC_motor_controller();
    DC_motor_controller* rightDown = new DC_motor_controller();
    DC_motor_controller* leftUp = new DC_motor_controller();
    DC_motor_controller* leftDown = new DC_motor_controller();

    FourMotors(DC_motor_controller * m1, DC_motor_controller* m2, DC_motor_controller* m3, DC_motor_controller*m4);
    void together(float vel);
    void together(float vel, float rot);
    void together(float vel1, float rot1, float vel2, float rot2, float vel3, float rot3 , float vel4, float rot4);
    void setGyreDegreesRatio(float rot, float ang);
    void turnDegree(float rot, float ang);
    void stop(unsigned int t);
    void reset();
    void run(int pwm);
    void run(int pwm1, int pwm2, int pwm3, int pwm4);

  private:
    float rot_per_degree;
    void resetMotors();

};

#endif
