#ifndef ema_filter_h
#define ema_filter_h

#include"abstract_filter.h"

class EMAFilter : public AbstractFilter {
private:
    float alpha; // Коэффициент сглаживания (0 < alpha <= 1)
    float filteredSpeedEMA = 0;
    bool firstValue = true;

public:
    EMAFilter(float a=0.05){
        alpha=a;
    }

    // Функция для вычисления скользящего среднего
    float update(float newValue) {
        if (firstValue) {
            filteredSpeedEMA = newValue;
            firstValue = false;
        } else {
            // EMA формула: filtered = alpha * new + (1 - alpha) * filtered
            filteredSpeedEMA = (alpha * newValue + (1 - alpha) * filteredSpeedEMA);
        }
        return filteredSpeedEMA;
    }
};

#endif