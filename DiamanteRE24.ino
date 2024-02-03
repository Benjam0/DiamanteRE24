#include <Wire.h>
#include <FourMotors.h>
#include <DC_motor_controller.h>
#include <Servo.h>
#include <Adafruit_TCS34725softi2c.h>
#include <VL53L0X.h>
#include <LiquidCrystal.h>

DC_motor_controller motorFR, motorBR, motorFL, motorBL;
FourMotors motor = FourMotors(&motorFR, &motorBR, &motorFL, &motorBL);
Servo servo1, servo2, servo3, servo4, servo5;

Adafruit_TCS34725softi2c tcsR = Adafruit_TCS34725softi2c
(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X, 8, 9);
Adafruit_TCS34725softi2c tcsM = Adafruit_TCS34725softi2c
(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X, 10, 11);
Adafruit_TCS34725softi2c tcsL = Adafruit_TCS34725softi2c
(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X, 12, 13);

VL53L0X sensorVL;

int
  trig1 = 34, echo1 = 35,
  trig2 = 36, echo2 = 37,
  trig3 = 38, echo3 = 39,
  trig4 = 40, echo4 = 41;
float
  distancia1,
  distancia2,
  distancia3,
  distancia4;
long 
  tempo1,
  tempo2,
  tempo3,
  tempo;

LiquidCrystal lcd(A5, A6, A7, A8, A9, A10);// rs, en, d4, d5, d6, d7

void setup() {
  Wire.begin();
  Serial.begin(9600);
  lcd.begin(16, 2);

  motorFR.hBridge(26, 27, 4); //IN1, IN2, ENA
  motorFL.hBridge(28, 29, 5); //IN3, IN4, ENB
  motorBR.hBridge(30, 31, 6); //IN1, IN2, ENA
  motorBL.hBridge(32, 33, 7); //IN3, IN4, ENB
  motorFR.setEncoderPin(2, 22); 
  motorFL.setEncoderPin(3, 23); 
  motorBR.setEncoderPin(18, 24); 
  motorBL.setEncoderPin(19, 25); 
  attachInterrupt(digitalPinToInterrupt(2), interruptMotorFR, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), interruptMotorFL, FALLING);
  attachInterrupt(digitalPinToInterrupt(18), interruptMotorBR, FALLING);
  attachInterrupt(digitalPinToInterrupt(19), interruptMotorBL, FALLING);
  motorFR.setRR(21);
  motorFL.setRR(21);
  motorBR.setRR(21);
  motorBL.setRR(21);
  motorFR.setPPR(11);
  motorFL.setPPR(11);
  motorBR.setPPR(11);
  motorBL.setPPR(11);
  // precisamos testar até encontrar bons valores de kp, ki e kd
  motorFR.setPIDconstants(1.2, 0.2, 0.015); //kp, ki, kd
  motorFL.setPIDconstants(1.2, 0.2, 0.015); //kp, ki, kd
  motorBR.setPIDconstants(1.2, 0.2, 0.015); //kp, ki, kd
  motorBL.setPIDconstants(1.2, 0.2, 0.015); //kp, ki, kd
  motorFR.setPins(); 
  motorFL.setPins(); 
  motorBR.setPins(); 
  motorBL.setPins(); 
  motorFR.stop();
  motorFL.stop();
  motorBR.stop();
  motorBL.stop();

  servo1.attach(A0);
  servo1.attach(A1);
  servo1.attach(A2);
  servo1.attach(A3);
  servo1.attach(A4);


  if (tcsR.begin() && tcsM.begin() && tcsL.begin()) {
    Serial.println("Found sensors");
  } else {
    Serial.println("No TCS34725s found ... check your connections");
    while (1);
  }

  sensorVL.setTimeout(500);
  if (!sensorVL.init())
  {
    Serial.println("Failed to detect and initialize VL sensor!");
    while (1) {}
  }
  sensorVL.startContinuous();
  
}

void loop() {
  uint16_t r, g, b, c, colorTemp, lux;
  tcsR.getRawData(&r, &g, &b, &c);
  colorTemp = tcsR.calculateColorTemperature(r, g, b);
  lux = tcsR.calculateLux(r, g, b);
  tcsM.getRawData(&r, &g, &b, &c);
  colorTemp = tcsM.calculateColorTemperature(r, g, b);
  lux = tcsM.calculateLux(r, g, b);
  tcsL.getRawData(&r, &g, &b, &c);
  colorTemp = tcsL.calculateColorTemperature(r, g, b);
  lux = tcsL.calculateLux(r, g, b);
  
  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); 
  Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  Serial.print(sensorVL.readRangeContinuousMillimeters());
  if (sensorVL.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();

}

void interruptMotorFR () {   
  motorFR.isr();              
}
void interruptMotorFL () {   
  motorFL.isr();              
}
void interruptMotorBR () {   
  motorBR.isr();              
}
void interruptMotorBL () {   
  motorBL.isr();              
}

/* 
Portas utilizadas:
2  8   18  26  32  38  A2  A8   
3  9   19  27  33  39  A3  A9   
4  10  22  28  34  40  A4  A10  
5  11  23  29  35  41  A5  
6  12  24  30  36  A0  A6  
7  13  25  31  37  A1  A7  

O que temos até então:
1 arduino mega
4 motores encoder jga25
2 ponte h l298n
5 servos sg90
3 sensores tcs34725(r, m, l)
1 sensor vl53l0x
4 sensores sr04
1 lcd 16x2
*/

