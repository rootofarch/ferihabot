#ifndef ODOMETER_H
#define ODOMETER_H

class Odometer {
public:
  Odometer();
  long oneTurnTick = 12000;
  float oneTurnDistance = 38.32743037;
  long rightEncoderTick = 0;
  long leftEncoderTick = 0;
  float distanceDriven = 0;

  float getWay();

};


#endif
