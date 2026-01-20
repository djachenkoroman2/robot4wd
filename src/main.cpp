#include <Arduino.h>
#include"pins.h"
#include"abstract_filter.h"
#include"simple_kalman_filter.h"
#include"ma_filter.h"
#include"PID.h"

#define SERIALSPEED 9600
#define DT 100

volatile unsigned int rot = 0;
unsigned long speedRPM = 0;
unsigned long lastCalcTime = 0;
unsigned long currentTime = 0;
float filteredSpeed = 0;

SimpleKalmanFilter kalmanFilter(0.01, 0.7, 0); // Q=0.01, R=0.1
MovingAverageFilter MAFilter(30); 

AbstractFilter& filter1=MAFilter; 
AbstractFilter& filter2=kalmanFilter; 


void detect() {
    rot++;
}

void setup() {

    Serial.begin(SERIALSPEED); // Увеличим скорость для отладки
    
    pinMode(PIN_SPD_SENSOR_1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_SENSOR_1), detect, RISING);

    lastCalcTime = millis();
}

void loop() {

    currentTime = millis();
    if (currentTime - lastCalcTime >= SPEED_TIME_INTERVAL) 
    {
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
            filteredSpeed = filter2.update(filter1.update(speedRPM));
        }
        
        lastCalcTime = currentTime;
    }



    // Вывод значений
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= SPEED_TIME_INTERVAL) {
        Serial.print(speedRPM);
        Serial.print(" ");
        Serial.println(filteredSpeed, 1); 
        lastPrintTime = currentTime;
    }
    delay(1);
}
