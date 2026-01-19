#ifndef ma_combo_filter_h
#define ma_combo_filter_h

#include"abstract_filter.h"

class  MAComboFilter : public AbstractFilter {

private:
    int window_size;    

    const int filter_window = 55; // Нечетное число для медианы
    float * filterBuffer;
    int filterIndex = 0;
    
    // Функция для сортировки (для медианного фильтра)
    void bubbleSort(float arr[], int n) {
        for (int i = 0; i < n-1; i++) {
            for (int j = 0; j < n-i-1; j++) {
                if (arr[j] > arr[j+1]) {
                    float temp = arr[j];
                    arr[j] = arr[j+1];
                    arr[j+1] = temp;
                }
            }
        }
    }
public:
    MAComboFilter(int ws=10){
        window_size=ws;
        filterBuffer = new float[window_size];
    }

    float update(float newValue) {
        // Добавляем новое значение в буфер
        filterBuffer[filterIndex] = newValue;
        filterIndex = (filterIndex + 1) % window_size;
        // Создаем копию буфера для сортировки
        float * sortedBuffer=new float[window_size];
        for (int i = 0; i < window_size; i++) {
            sortedBuffer[i] = filterBuffer[i];
        }
        // Сортируем для нахождения медианы
        bubbleSort(sortedBuffer, window_size);
        // Берем медиану (средний элемент)
        float median = sortedBuffer[window_size / 2];
        // Затем вычисляем среднее вокруг медианы (можно вернуть просто медиану)
        float sum = 0;
        for (int i = 0; i < window_size; i++) {
            sum += filterBuffer[i];
        }
        return sum / window_size; // или return median;
    }
};


#endif
