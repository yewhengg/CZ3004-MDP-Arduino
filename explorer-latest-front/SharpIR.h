#ifndef SharpIR_h
#define SharpIR_h

#include "Arduino.h"

class SharpIR {
  public:
    SharpIR(int irPin, const uint32_t sensorType, double m, double b, double k);
    static const uint32_t GP2Y0A21YK0F = 1080;
    static const uint32_t GP2Y0A02YK0F = 20150;
    static const int SRVMin = 120;
    static const int LRVMin = 160;
    static const int sampleSize = 50;
    int getRaw() const;
    double distance() const;
    void sort(int a[], int size) const;
 private:
  int _irPin;
  uint32_t _model;
  double _m;
  double _b;
  double _k;
};

#endif
