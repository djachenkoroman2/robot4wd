#include <Arduino.h>
#include"pins.h"

#include"abstract_filter.h"

#include"simple_kalman_filter.h"
#include"ma_filter.h"
#include"ema_filter.h"
#include"ma_combo_filter.h"

volatile unsigned int rot = 0;
volatile unsigned long lastTime = 0;
unsigned long speedRPM = 0;
unsigned long lastCalcTime = 0;

// Создаем экземпляр фильтра Калмана
SimpleKalmanFilter kalmanFilter(0.005, 0.99, 0); // Q=0.01, R=0.1
MovingAverageFilter MAFilter(20); 
EMAFilter emafilter(0.05); 

AbstractFilter& filter1=kalmanFilter; 
AbstractFilter& filter2=MAFilter; 
AbstractFilter& filter3=emafilter; 

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

    static float filteredSpeed1 = 0;    
    static float filteredSpeed2 = 0;
    static float filteredSpeed3 = 0;
    
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
            
            // Применяем фильтр 
            filteredSpeed1 = filter1.update(speedRPM);
            filteredSpeed2 = filter2.update(speedRPM);
            filteredSpeed3 = filter3.update(speedRPM);
        }
        
        lastCalcTime = currentTime;
    }
    
    // Вывод значений
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 100) {
        Serial.print(speedRPM);
        Serial.print(" ");
        Serial.print(filteredSpeed1, 1); 
        Serial.print(" ");
        Serial.print(filteredSpeed2, 1); 
        Serial.print(" ");
        Serial.println(filteredSpeed3, 1); 
        
        lastPrintTime = currentTime;
    }
    
    delay(1);
}
