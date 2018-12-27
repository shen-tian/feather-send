#ifndef IMU_H
#define IMU_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

class Imu {
  private:
  Adafruit_BNO055 _bno = Adafruit_BNO055();
  float _magDeclination;
  uint8_t _sensorOffset;
  long _lastMovement;


  public:
  imu::Vector<3> euler, mag, gyro;
  int8_t temperature;
  int32_t heading;
  uint8_t sysCal, gyroCal, accCal, magCal;

  Imu(float magDeclination, uint8_t sensorOffset) {
    _magDeclination = magDeclination;
    _sensorOffset = sensorOffset;
    _lastMovement = 0;
  }

  void init() {
    Serial.println("IMU init: ");
    while(!_bno.begin()) {
    }
    delay(100);
    _bno.setExtCrystalUse(true);
  }

  void update() {
    euler = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    mag = _bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    gyro = _bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    temperature = _bno.getTemp();

    heading = euler.x() + _magDeclination + _sensorOffset;
    if (heading > 360)
      heading -= 360;

    if (abs(gyro.x()) > 2)
      _lastMovement = millis();

    _bno.getCalibration(&sysCal, &gyroCal, &accCal, &magCal);
  }

  bool isStill() {
    return millis() - _lastMovement > 1000;
  }
};

#endif
