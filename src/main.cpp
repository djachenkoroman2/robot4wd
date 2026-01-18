#include <Arduino.h>
#include"pins.h"
#include"abstract_filter.h"
#include"simple_kalman_filter.h"
#include"moving_average_filter.h"



// const byte interruptPin = 2;
volatile unsigned int rot = 0;
volatile unsigned long lastTime = 0;
unsigned long speedRPM = 0;
unsigned long lastCalcTime = 0;


// Создаем экземпляр фильтра Калмана
SimpleKalmanFilter kalmanFilter(0.005, 0.99, 0); // Q=0.01, R=0.1
MovingAverageFilter MAFilter(20); 

AbstractFilter& filter=kalmanFilter; 

void detect() {
    unsigned long currentTime = millis();
    
    // Антидребезг
    if (currentTime - lastTime > 10) {
        rot++;
        lastTime = currentTime;
    }
}

void setup() {

    Serial.begin(9600); // Увеличим скорость для отладки
    
    pinMode(PIN_SPD_SENSOR_1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_SENSOR_1), detect, RISING);
    
    lastTime = millis();
    lastCalcTime = millis();
    
    Serial.println("Система измерения скорости с фильтром Калмана");
    Serial.println("Формат: Raw,Filtered");
}

void loop() {
    unsigned long currentTime = millis();
    static float filteredSpeed = 0;
    
    // Вычисляем скорость каждые 100 мс
    if (currentTime - lastCalcTime >= 100) {
        // Отключаем прерывания на время чтения/сброса переменной
        noInterrupts();
        unsigned int rotations = rot;
        rot = 0;
        interrupts();
        
        // Вычисляем RPM
        if (lastCalcTime > 0) {
            unsigned long timeDiff = currentTime - lastCalcTime;
            if (timeDiff > 0 && rotations > 0) {
                speedRPM = (rotations * 60000UL) / timeDiff;
            } else {
                speedRPM = 0;
            }
            
            // Применяем фильтр Калмана
            // filteredSpeed = kalmanFilter.update(speedRPM);
            filteredSpeed = filter.update(speedRPM);
        }
        
        lastCalcTime = currentTime;
    }
    
    // Вывод значений
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 100) {
        Serial.print(speedRPM);
        Serial.print(" ");
        Serial.println(filteredSpeed, 1); // 1 знак после запятой
        
        lastPrintTime = currentTime;
    }
    
    delay(1);
}
