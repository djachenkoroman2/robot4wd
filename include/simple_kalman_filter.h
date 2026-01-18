#ifndef simple_kalman_filter_h
#define simple_kalman_filter_h

#include"abstract_filter.h"


class SimpleKalmanFilter : public AbstractFilter {
private:
    float Q; // Ковариация процесса (шум модели)
    float R; // Ковариация измерения (шум датчика)
    float P; // Ошибка оценки
    float X; // Текущая оценка
    float K; // Коэффициент Калмана
    
public:
    // Конструктор
    SimpleKalmanFilter(float q, float r, float initial_value = 0) {
        Q = q;
        R = r;
        P = 1.0;
        X = initial_value;
    }
    
    // Обновление фильтра с новым измерением
    float update(float measurement) {
        // Этап предсказания
        P = P + Q;
        
        // Этап обновления
        K = P / (P + R);
        X = X + K * (measurement - X);
        P = (1 - K) * P;
        
        return X;
    }
    
    // Получить текущую оценку
    float getValue() {
        return X;
    }
    
    // Сброс фильтра
    void reset(float value) {
        P = 1.0;
        X = value;
    }
};

#endif