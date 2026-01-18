#ifndef ma_filter_h
#define ma_filter_h

#include"abstract_filter.h"

class MovingAverageFilter : public AbstractFilter {
private:
    int window_size;    
    float * speedBuffer; // Буфер значений
    int bufferIndex = 0; // Текущий индекс в буфере
    bool bufferFilled = false; // Флаг заполнения буфера

public:
    MovingAverageFilter(int ws=10){
        window_size=ws;
        speedBuffer = new float[window_size];
    }

    // Функция для вычисления скользящего среднего
    float update(float newValue) {
        // Добавляем новое значение в буфер
        speedBuffer[bufferIndex] = newValue;
        bufferIndex = (bufferIndex + 1) % window_size;
        
        // Отмечаем, что буфер заполнен после первого полного прохода
        if (bufferIndex == 0) {
            bufferFilled = true;
        }
        
        // Вычисляем среднее значение
        float sum = 0;
        int count = bufferFilled ? window_size : bufferIndex;
        
        for (int i = 0; i < count; i++) {
            sum += speedBuffer[i];
        }
        
        return (count > 0) ? (sum / count) : newValue;
    }


};

#endif