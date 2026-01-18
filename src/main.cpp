#include <Arduino.h>
#include"abstract_filter.h"
#include"pins.h"

#define TIMER1 6000

struct SpeedSensorPar
{
    unsigned int rot = 0;
    unsigned long int tm;
    unsigned long int spd = 0;
    unsigned int dt = 0;
};

SpeedSensorPar spd_sens2;
SpeedSensorPar spd_sens1;



long state=1;

long Timer1=0;

void detect1() {
    spd_sens1.rot++; // прибавляем единичку к счётчику обротов
    spd_sens1.dt = millis() - spd_sens1.tm; // вычисляем время с последнего расчёта
    if( spd_sens1.dt >= 100 )
    { // если прошло 100мс или более, то начинаем расчёт
        spd_sens1.spd = spd_sens1.rot*60000/spd_sens1.dt;
        spd_sens1.rot = 0; // обнуляем счётчик
        spd_sens1.tm = millis(); // запоминаем время расчёта
    }
}

void detect2() {
    spd_sens2.rot++; // прибавляем единичку к счётчику обротов
    spd_sens2.dt = millis() - spd_sens2.tm; // вычисляем время с последнего расчёта
    if( spd_sens2.dt >= 100 )
    { // если прошло 100мс или более, то начинаем расчёт
        spd_sens2.spd = spd_sens2.rot*60000/spd_sens2.dt;
        spd_sens2.rot = 0; // обнуляем счётчик
        spd_sens2.tm = millis(); // запоминаем время расчёта
    }
}

void setup() {

    Serial.begin(9600);
  
    pinMode(PIN_SPD_SENSOR_2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_SENSOR_2), detect2, RISING);

    pinMode(PIN_SPD_SENSOR_1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_SENSOR_1), detect1, RISING);

    spd_sens1.tm = millis();
    spd_sens2.tm = millis();

    pinMode( PIN_MOTORDRV1_ENA, OUTPUT );
    pinMode( PIN_MOTORDRV1_IN1, OUTPUT );
    pinMode( PIN_MOTORDRV1_IN2, OUTPUT );

    pinMode( PIN_MOTORDRV1_ENB, OUTPUT );
    pinMode( PIN_MOTORDRV1_IN3, OUTPUT );
    pinMode( PIN_MOTORDRV1_IN4, OUTPUT );

    Timer1=millis();
}
void loop() {

    if (state==1)
    {
        // выставляем 100% мощность на моторе А - 255 из 255
        analogWrite( PIN_MOTORDRV1_ENB, 255 );
        // выставляем режим мотора - вращение по часовой
        digitalWrite( PIN_MOTORDRV1_IN3, HIGH );
        digitalWrite( PIN_MOTORDRV1_IN4, LOW );

        // выставляем 100% мощность на моторе А - 255 из 255
        analogWrite( PIN_MOTORDRV1_ENA, 255 );
        // выставляем режим мотора - вращение по часовой
        digitalWrite( PIN_MOTORDRV1_IN2, HIGH );
        digitalWrite( PIN_MOTORDRV1_IN1, LOW );


        Serial.print(spd_sens1.spd);
        Serial.print(" ");
        Serial.println(spd_sens2.spd);
        delay(100);
    }

    if (state==-1)
    {
        // выставляем мощность на мотора А - 150 из 255
        analogWrite( PIN_MOTORDRV1_ENB, 150 );
        // режим мотора - вращение против часовой
        digitalWrite( PIN_MOTORDRV1_IN3, LOW );
        digitalWrite( PIN_MOTORDRV1_IN4, HIGH );
        
        // выставляем мощность на мотора А - 150 из 255
        analogWrite( PIN_MOTORDRV1_ENA, 150 );
        // режим мотора - вращение против часовой
        digitalWrite( PIN_MOTORDRV1_IN2, LOW );
        digitalWrite( PIN_MOTORDRV1_IN1, HIGH );

        
        Serial.print(spd_sens1.spd);
        Serial.print(" ");
        Serial.println(spd_sens2.spd);
        delay(100); // пауза 3сек
    }

    if (millis()-Timer1>=TIMER1)
    {
        state=-1*state;
        Timer1=millis();
    }
}