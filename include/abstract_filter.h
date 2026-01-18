// AbstractSensor.h
#ifndef abstract_filter_h
#define abstract_filter_h

class AbstractFilter {
  public:
    // Обычная виртуальная функция (может быть переопределена)
    virtual float update(float measurement);
};

#endif