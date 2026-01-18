// AbstractSensor.h
#ifndef AbstractFilter_h
#define AbstractFilter_h

class AbstractFilter {
  public:
    // Обычная виртуальная функция (может быть переопределена)
    virtual float update(float measurement);
};

#endif