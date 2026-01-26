#include <Arduino.h>
#include"pins.h"
#include"abstract_filter.h"
#include"simple_kalman_filter.h"
#include"ma_filter.h"
#include"PID.h"

#define SERIALSPEED 9600
#define DT 150
#define DT2 500
#define SPD_TARGET 1500

volatile unsigned int rot = 0;
unsigned long speedRPM = 0;
unsigned long lastCalcTime = 0;
unsigned long lastCalcTime2 = 0;
unsigned long currentTime = 0;
unsigned long currentTime2 = 0;

// float filteredSpeed = 0;

// SimpleKalmanFilter kalmanFilter(0.01, 0.7, 0); // Q=0.01, R=0.1
MovingAverageFilter MAFilter(10); 

AbstractFilter& filter1=MAFilter; 
// AbstractFilter& filter2=kalmanFilter; 

// PID pidController(20.0, 0.0, 0.0, DT2/1000); // Kp=2, Ki=5, Kd=1, время дискретизации 100 мс
// float Kp, Ki, Kd;
// float integral, pError;


// float computePID(float setpoint, float input1, float dt) {
//     float input;
//     if (isnan(input1)) input=0; else input = input1;
//     float error = setpoint - input;
//     // float derivative = (error - pError) / dt;
//     pError = error;
//     integral += error * dt;
//     // return error * Kp + integral * Ki + derivative * Kd;
//     return error * Kp + integral * Ki;
// }


void detect() {
    rot++;
}

void setup() {

    pinMode( PIN_MOTORDRV1_ENA, OUTPUT );
    pinMode( PIN_MOTORDRV1_IN1, OUTPUT );
    pinMode( PIN_MOTORDRV1_IN2, OUTPUT );
    pinMode(PIN_SPD_SENSOR_1, INPUT_PULLUP);

    Serial.begin(SERIALSPEED); // Увеличим скорость для отладки

    attachInterrupt(digitalPinToInterrupt(PIN_SPD_SENSOR_1), detect, RISING);

    // Настройка ПИД-регулятора
    // pidController.setTarget(SPD_TARGET);
    // pidController.setOutputLimits(0, 255);  // Для ШИМ

    // Kp = 2;
    // Ki = 5;
    // Kd = 0;
    // pError = 0;
    // integral = 0;

    lastCalcTime2=lastCalcTime = millis();
}

void loop() {

    // Управление нагревателем через ШИМ
    analogWrite(PIN_MOTORDRV1_ENA, 130);
    digitalWrite( PIN_MOTORDRV1_IN1, HIGH );
    digitalWrite( PIN_MOTORDRV1_IN2, LOW );

    currentTime = millis();
    if (currentTime - lastCalcTime >= DT) 
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
                // filteredSpeed = filter2.update(filter1.update(rotations / timeDiff));
                speedRPM = int(filter1.update(rotations));
                
            } else {
                speedRPM = 0;
            }
        }
        
        lastCalcTime = currentTime;
    }

    currentTime2 = millis();
    if (currentTime2 - lastCalcTime2 >= DT2)
    {
        // Вычисление ПИД
        // float output = pidController.compute(filteredSpeed);
        // float output = computePID(SPD_TARGET, speedRPM, DT2/1000);
        // int val = map(output,0,3600,0,255);

        Serial.println(speedRPM);
        // Serial.print(" ");
        // Serial.print(SPD_TARGET);
        // Serial.print(" ");
        // Serial.println(output, 1); 
        lastCalcTime2 = currentTime2;
    }
    delay(1);
}
