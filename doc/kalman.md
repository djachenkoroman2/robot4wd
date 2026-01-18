## Фильтр Калмана
Добавим фильтр Калмана для сглаживания показаний скорости. Вот полный код с реализацией:

```cpp
#include <Arduino.h>

const byte interruptPin = 2;
volatile unsigned int rot = 0;
volatile unsigned long lastTime = 0;
unsigned long speedRPM = 0;
unsigned long lastCalcTime = 0;

// ==================== КЛАСС ФИЛЬТРА КАЛМАНА ====================
class SimpleKalmanFilter {
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

// Создаем экземпляр фильтра Калмана
SimpleKalmanFilter kalmanFilter(0.01, 0.1, 0); // Q=0.01, R=0.1

void detect() {
    unsigned long currentTime = millis();
    
    // Антидребезг
    if (currentTime - lastTime > 10) {
        rot++;
        lastTime = currentTime;
    }
}

void setup() {
    Serial.begin(115200); // Увеличим скорость для отладки
    
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), detect, RISING);
    
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
            filteredSpeed = kalmanFilter.update(speedRPM);
        }
        
        lastCalcTime = currentTime;
    }
    
    // Вывод значений
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 100) {
        Serial.print(speedRPM);
        Serial.print(",");
        Serial.println(filteredSpeed, 1); // 1 знак после запятой
        
        lastPrintTime = currentTime;
    }
    
    delay(1);
}
```

## Улучшенная версия с адаптивными параметрами:

```cpp
#include <Arduino.h>

const byte interruptPin = 2;
volatile unsigned int rot = 0;
volatile unsigned long lastTime = 0;
unsigned long speedRPM = 0;
unsigned long lastCalcTime = 0;

// ==================== УЛУЧШЕННЫЙ ФИЛЬТР КАЛМАНА ====================
class AdvancedKalmanFilter {
private:
    float Q;     // Ковариация процесса
    float R;     // Ковариация измерения
    float P;     // Ошибка оценки
    float X;     // Текущая оценка
    float K;     // Коэффициент Калмана
    float lastMeasurement;
    unsigned long lastUpdateTime;
    float velocity; // Оценка скорости изменения
    
public:
    AdvancedKalmanFilter(float q, float r, float initial_value = 0) {
        Q = q;
        R = r;
        P = 1.0;
        X = initial_value;
        velocity = 0;
        lastMeasurement = initial_value;
        lastUpdateTime = millis();
    }
    
    float update(float measurement) {
        unsigned long currentTime = millis();
        float dt = (currentTime - lastUpdateTime) / 1000.0; // В секундах
        
        if (dt <= 0) dt = 0.1; // Защита от деления на ноль
        
        // Адаптивный параметр Q в зависимости от изменения сигнала
        float innovation = abs(measurement - lastMeasurement);
        float adaptiveQ = Q * (1.0 + innovation / 100.0);
        
        // Этап предсказания с учетом скорости
        X = X + velocity * dt;
        P = P + adaptiveQ;
        
        // Этап обновления
        K = P / (P + R);
        float residual = measurement - X;
        X = X + K * residual;
        
        // Обновляем оценку скорости
        velocity = velocity + (K * residual / dt) * 0.1;
        
        P = (1 - K) * P;
        
        lastMeasurement = measurement;
        lastUpdateTime = currentTime;
        
        return X;
    }
    
    float getValue() {
        return X;
    }
    
    float getVelocity() {
        return velocity;
    }
    
    void reset(float value) {
        P = 1.0;
        X = value;
        velocity = 0;
        lastMeasurement = value;
        lastUpdateTime = millis();
    }
};

// Создаем экземпляр улучшенного фильтра
AdvancedKalmanFilter kalmanFilter(0.01, 0.5, 0);

void detect() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastTime > 10) {
        rot++;
        lastTime = currentTime;
    }
}

void setup() {
    Serial.begin(115200);
    
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), detect, RISING);
    
    lastTime = millis();
    lastCalcTime = millis();
    
    Serial.println("Система измерения скорости с улучшенным фильтром Калмана");
    Serial.println("Формат: Raw,Filtered,Velocity");
}

void loop() {
    unsigned long currentTime = millis();
    static float filteredSpeed = 0;
    
    if (currentTime - lastCalcTime >= 100) {
        noInterrupts();
        unsigned int rotations = rot;
        rot = 0;
        interrupts();
        
        if (lastCalcTime > 0) {
            unsigned long timeDiff = currentTime - lastCalcTime;
            if (timeDiff > 0 && rotations > 0) {
                speedRPM = (rotations * 60000UL) / timeDiff;
                
                // Фильтруем явные выбросы
                if (speedRPM > 5000) { // Максимальная разумная скорость
                    speedRPM = 0;
                }
            } else {
                speedRPM = 0;
            }
            
            filteredSpeed = kalmanFilter.update(speedRPM);
        }
        
        lastCalcTime = currentTime;
    }
    
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 100) {
        Serial.print(speedRPM);
        Serial.print(",");
        Serial.print(filteredSpeed, 1);
        Serial.print(",");
        Serial.println(kalmanFilter.getVelocity(), 2);
        
        lastPrintTime = currentTime;
    }
    
    // Автосброс фильтра при длительном отсутствии вращений
    static unsigned long lastMovementTime = millis();
    if (speedRPM > 10) {
        lastMovementTime = currentTime;
    }
    
    if (currentTime - lastMovementTime > 5000) { // 5 секунд без движения
        kalmanFilter.reset(0);
        lastMovementTime = currentTime;
    }
    
    delay(1);
}
```

## Версия с двойным фильтром (Калман + медианный префильтр):

```cpp
#include <Arduino.h>

const byte interruptPin = 2;
volatile unsigned int rot = 0;
volatile unsigned long lastTime = 0;
unsigned long speedRPM = 0;
unsigned long lastCalcTime = 0;

// ==================== МЕДИАННЫЙ ПРЕФИЛЬТР ====================
class MedianFilter {
private:
    static const int SIZE = 5;
    unsigned long buffer[SIZE];
    int index;
    
public:
    MedianFilter() {
        index = 0;
        for (int i = 0; i < SIZE; i++) {
            buffer[i] = 0;
        }
    }
    
    unsigned long filter(unsigned long newValue) {
        buffer[index] = newValue;
        index = (index + 1) % SIZE;
        
        // Сортировка пузырьком для нахождения медианы
        unsigned long tempBuffer[SIZE];
        for (int i = 0; i < SIZE; i++) tempBuffer[i] = buffer[i];
        
        for (int i = 0; i < SIZE-1; i++) {
            for (int j = 0; j < SIZE-i-1; j++) {
                if (tempBuffer[j] > tempBuffer[j+1]) {
                    unsigned long temp = tempBuffer[j];
                    tempBuffer[j] = tempBuffer[j+1];
                    tempBuffer[j+1] = temp;
                }
            }
        }
        
        return tempBuffer[SIZE/2];
    }
};

// ==================== ФИЛЬТР КАЛМАНА ====================
class KalmanFilter {
private:
    float Q, R, P, X, K;
    
public:
    KalmanFilter(float q, float r, float initial_value = 0) {
        Q = q;
        R = r;
        P = 1.0;
        X = initial_value;
    }
    
    float update(float measurement) {
        // Предсказание
        P = P + Q;
        
        // Обновление
        K = P / (P + R);
        X = X + K * (measurement - X);
        P = (1 - K) * P;
        
        return X;
    }
    
    float getValue() { return X; }
    
    void reset(float value) {
        P = 1.0;
        X = value;
    }
};

// Создаем фильтры
MedianFilter medianFilter;
KalmanFilter kalmanFilter(0.01, 0.2, 0);

void detect() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime > 10) {
        rot++;
        lastTime = currentTime;
    }
}

void setup() {
    Serial.begin(115200);
    
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), detect, RISING);
    
    lastTime = millis();
    lastCalcTime = millis();
    
    Serial.println("Двойной фильтр: медиана + Калман");
    Serial.println("Формат: Raw,Median,Kalman");
}

void loop() {
    unsigned long currentTime = millis();
    static float kalmanSpeed = 0;
    
    if (currentTime - lastCalcTime >= 100) {
        noInterrupts();
        unsigned int rotations = rot;
        rot = 0;
        interrupts();
        
        if (lastCalcTime > 0) {
            unsigned long timeDiff = currentTime - lastCalcTime;
            if (timeDiff > 0 && rotations > 0) {
                speedRPM = (rotations * 60000UL) / timeDiff;
            } else {
                speedRPM = 0;
            }
            
            // Применяем двойную фильтрацию
            unsigned long medianFiltered = medianFilter.filter(speedRPM);
            kalmanSpeed = kalmanFilter.update(medianFiltered);
        }
        
        lastCalcTime = currentTime;
    }
    
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 100) {
        Serial.print(speedRPM);
        Serial.print(",");
        // Чтобы получить медианное значение, нужно сохранять его
        static unsigned long lastMedian = 0;
        lastMedian = medianFilter.filter(speedRPM);
        Serial.print(lastMedian);
        Serial.print(",");
        Serial.println(kalmanSpeed, 1);
        
        lastPrintTime = currentTime;
    }
    
    delay(1);
}
```

## Настройка параметров фильтра Калмана:

```cpp
// Q - ковариация процесса (шум модели)
// R - ковариация измерения (шум датчика)

// Для быстро меняющегося сигнала:
// kalmanFilter(0.1, 1.0)  // Большой Q, малый R - быстрое отслеживание

// Для плавного сигнала:
// kalmanFilter(0.001, 10.0) // Малый Q, большой R - сильное сглаживание

// Универсальные настройки:
// kalmanFilter(0.01, 0.5)  // Компромиссный вариант
```

## Советы по использованию:

1. **Калибровка параметров**:
   - Увеличивайте `R` для большего сглаживания
   - Увеличивайте `Q` для более быстрого отслеживания изменений

2. **Визуализация**:
   - Используйте Serial Plotter в Arduino IDE для наглядного сравнения сырых и отфильтрованных данных

3. **Адаптация**:
   - Можете динамически менять `R` в зависимости от уровня шума
   - При резких изменениях скорости временно увеличивайте `Q`

Начните с параметров `Q=0.01, R=0.5` и регулируйте их в зависимости от поведения вашей системы.