#ifndef ma_combo_filter_h
#define ma_combo_filter_h

#include"abstract_filter.h"

class  MAComboFilter : public AbstractFilter {

private:
    int window_size;    
    float * speedBuffer; // Буфер значений
    int bufferIndex = 0; // Текущий индекс в буфере
    bool bufferFilled = false; // Флаг заполнения буфера

    const int filter_window = 55; // Нечетное число для медианы
    // unsigned long filterBuffer[FILTER_WINDOW];
    int filterIndex = 0;
    
    // Функция для сортировки (для медианного фильтра)
    void bubbleSort(unsigned long arr[], int n) {
        for (int i = 0; i < n-1; i++) {
            for (int j = 0; j < n-i-1; j++) {
                if (arr[j] > arr[j+1]) {
                    unsigned long temp = arr[j];
                    arr[j] = arr[j+1];
                    arr[j+1] = temp;
                }
            }
        }
    }
public:
    MAComboFilter(int ws=10){
        window_size=ws;
        speedBuffer = new float[window_size];
    }
};


#endif
