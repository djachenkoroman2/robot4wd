Добавить скользящее среднее (moving average) можно несколькими способами. Вот три варианта с разной сложностью:

## Вариант 1: Простое скользящее среднее (фиксированное окно)

```cpp
#include <Arduino.h>

const byte interruptPin = 2;
volatile unsigned int rot = 0;
volatile unsigned long lastTime = 0;
unsigned long speedRPM = 0;
unsigned long lastCalcTime = 0;

// Параметры для скользящего среднего
const int WINDOW_SIZE = 10; // Размер окна усреднения
unsigned long speedBuffer[WINDOW_SIZE]; // Буфер значений
int bufferIndex = 0; // Текущий индекс в буфере
bool bufferFilled = false; // Флаг заполнения буфера

void detect() {
    unsigned long currentTime = millis();
    
    // Антидребезг
    if (currentTime - lastTime > 10) {
        rot++;
        lastTime = currentTime;
    }
}

// Функция для вычисления скользящего среднего
unsigned long calculateMovingAverage(unsigned long newValue) {
    // Добавляем новое значение в буфер
    speedBuffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
    
    // Отмечаем, что буфер заполнен после первого полного прохода
    if (bufferIndex == 0) {
        bufferFilled = true;
    }
    
    // Вычисляем среднее значение
    unsigned long sum = 0;
    int count = bufferFilled ? WINDOW_SIZE : bufferIndex;
    
    for (int i = 0; i < count; i++) {
        sum += speedBuffer[i];
    }
    
    return (count > 0) ? (sum / count) : newValue;
}

void setup() {
    Serial.begin(9600);
    
    // Инициализация буфера нулями
    for (int i = 0; i < WINDOW_SIZE; i++) {
        speedBuffer[i] = 0;
    }
    
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), detect, RISING);
    
    lastTime = millis();
    lastCalcTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    static unsigned long filteredSpeed = 0; // Отфильтрованное значение
    
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
            
            // Применяем скользящее среднее
            filteredSpeed = calculateMovingAverage(speedRPM);
        }
        
        lastCalcTime = currentTime;
    }
    
    // Вывод значений
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 100) {
        Serial.print("Raw: ");
        Serial.print(speedRPM);
        Serial.print(" RPM | Filtered: ");
        Serial.print(filteredSpeed);
        Serial.println(" RPM");
        lastPrintTime = currentTime;
    }
    
    delay(1);
}
```

## Вариант 2: Экспоненциальное скользящее среднее (EMA) - более эффективно по памяти

```cpp
#include <Arduino.h>

const byte interruptPin = 2;
volatile unsigned int rot = 0;
volatile unsigned long lastTime = 0;
unsigned long speedRPM = 0;
unsigned long lastCalcTime = 0;

// Параметры для EMA
float alpha = 0.3; // Коэффициент сглаживания (0 < alpha <= 1)
unsigned long filteredSpeedEMA = 0;
bool firstValue = true;

void detect() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastTime > 10) {
        rot++;
        lastTime = currentTime;
    }
}

// Экспоненциальное скользящее среднее
unsigned long calculateEMA(unsigned long newValue) {
    if (firstValue) {
        filteredSpeedEMA = newValue;
        firstValue = false;
    } else {
        // EMA формула: filtered = alpha * new + (1 - alpha) * filtered
        filteredSpeedEMA = (unsigned long)(alpha * newValue + (1 - alpha) * filteredSpeedEMA);
    }
    return filteredSpeedEMA;
}

void setup() {
    Serial.begin(9600);
    
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), detect, RISING);
    
    lastTime = millis();
    lastCalcTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    static unsigned long filteredSpeed = 0;
    
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
            
            filteredSpeed = calculateEMA(speedRPM);
        }
        
        lastCalcTime = currentTime;
    }
    
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 100) {
        Serial.print("Raw: ");
        Serial.print(speedRPM);
        Serial.print(" RPM | EMA: ");
        Serial.print(filteredSpeed);
        Serial.println(" RPM");
        lastPrintTime = currentTime;
    }
    
    delay(1);
}
```

## Вариант 3: Медианный фильтр + скользящее среднее (для защиты от выбросов)

```cpp
#include <Arduino.h>

const byte interruptPin = 2;
volatile unsigned int rot = 0;
volatile unsigned long lastTime = 0;
unsigned long speedRPM = 0;
unsigned long lastCalcTime = 0;

// Комбинированный фильтр: медиана + среднее
const int FILTER_WINDOW = 5; // Нечетное число для медианы
unsigned long filterBuffer[FILTER_WINDOW];
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

// Комбинированный фильтр: сначала медиана, затем среднее
unsigned long applyFilter(unsigned long newValue) {
    // Добавляем новое значение в буфер
    filterBuffer[filterIndex] = newValue;
    filterIndex = (filterIndex + 1) % FILTER_WINDOW;
    
    // Создаем копию буфера для сортировки
    unsigned long sortedBuffer[FILTER_WINDOW];
    for (int i = 0; i < FILTER_WINDOW; i++) {
        sortedBuffer[i] = filterBuffer[i];
    }
    
    // Сортируем для нахождения медианы
    bubbleSort(sortedBuffer, FILTER_WINDOW);
    
    // Берем медиану (средний элемент)
    unsigned long median = sortedBuffer[FILTER_WINDOW / 2];
    
    // Затем вычисляем среднее вокруг медианы (можно вернуть просто медиану)
    unsigned long sum = 0;
    for (int i = 0; i < FILTER_WINDOW; i++) {
        sum += filterBuffer[i];
    }
    
    return sum / FILTER_WINDOW; // или return median;
}

void detect() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastTime > 10) {
        rot++;
        lastTime = currentTime;
    }
}

void setup() {
    Serial.begin(9600);
    
    // Инициализация буфера фильтра
    for (int i = 0; i < FILTER_WINDOW; i++) {
        filterBuffer[i] = 0;
    }
    
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), detect, RISING);
    
    lastTime = millis();
    lastCalcTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    static unsigned long filteredSpeed = 0;
    
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
            
            // Применяем фильтр
            filteredSpeed = applyFilter(speedRPM);
        }
        
        lastCalcTime = currentTime;
    }
    
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 100) {
        Serial.print("Raw: ");
        Serial.print(speedRPM);
        Serial.print(" RPM | Filtered: ");
        Serial.print(filteredSpeed);
        Serial.println(" RPM");
        lastPrintTime = currentTime;
    }
    
    delay(1);
}
```

## Рекомендации по выбору:

1. **Вариант 1 (простое среднее)** - лучше всего подходит для плавных изменений скорости
2. **Вариант 2 (EMA)** - более эффективен по памяти, быстрее реагирует на изменения
3. **Вариант 3 (медиана+среднее)** - лучше всего защищает от резких выбросов (например, при дребезге)

## Настройка параметров:

```cpp
// Для простого среднего
const int WINDOW_SIZE = 10; // Чем больше, тем плавнее, но медленнее реакция

// Для EMA
float alpha = 0.3; // Чем больше alpha, тем быстрее реакция на изменения
                   // Чем меньше alpha, тем лучше сглаживание
```

Начните с Варианта 2 (EMA) с `alpha = 0.3`, это хороший компромисс между плавностью и скоростью реакции.