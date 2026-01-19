// PID.h
#ifndef pid_h
#define pid_h


class PID {
  private:
    // Параметры ПИД-регулятора
    float Kp;          // Пропорциональный коэффициент
    float Ki;          // Интегральный коэффициент  
    float Kd;          // Дифференциальный коэффициент
    
    // Переменные для вычислений
    float target;      // Целевое значение (уставка)
    float input;       // Текущее входное значение
    float output;      // Выходное значение регулятора
    
    // Внутренние переменные
    float integral;    // Интегральная сумма
    float prevError;   // Предыдущая ошибка
    float prevInput;   // Предыдущее входное значение (для производной по входу)
    
    // Ограничения
    float outMin;      // Минимальное выходное значение
    float outMax;      // Максимальное выходное значение
    
    // Временные параметры
    unsigned long lastTime;  // Время последнего вычисления
    float sampleTime;        // Время дискретизации (в секундах)
    
    // Режимы работы
    bool inAuto;       // Автоматический режим
    bool derivativeOnInput; // Производная по входу (true) или по ошибке (false)
    
  public:
    // Конструктор с параметрами по умолчанию
    PID(float kp = 1.0, float ki = 0.0, float kd = 0.0, float sampleTimeSec = 0.1) {
      Kp = kp;
      Ki = ki;
      Kd = kd;
      
      target = 0;
      input = 0;
      output = 0;
      
      integral = 0;
      prevError = 0;
      prevInput = 0;
      
      outMin = 0;
      outMax = 255;  // По умолчанию для ШИМ Arduino
      
      sampleTime = sampleTimeSec * 1000;  // Конвертируем в миллисекунды
      lastTime = millis() - sampleTime;
      
      inAuto = true;
      derivativeOnInput = true;  // Рекомендуется для уменьшения скачков
    }
    
    // Установка параметров ПИД
    void setParameters(float kp, float ki, float kd) {
      if (kp < 0 || ki < 0 || kd < 0) return;
      
      Kp = kp;
      Ki = ki;
      Kd = kd;
    }
    
    // Установка диапазона выходного сигнала
    void setOutputLimits(float min, float max) {
      if (min >= max) return;
      
      outMin = min;
      outMax = max;
      
      // Ограничиваем текущее выходное значение
      if (output > outMax) output = outMax;
      else if (output < outMin) output = outMin;
      
      // Ограничиваем интегральную сумму
      if (integral > outMax) integral = outMax;
      else if (integral < outMin) integral = outMin;
    }
    
    // Установка времени дискретизации
    void setSampleTime(float sampleTimeSec) {
      if (sampleTimeSec <= 0) return;
      
      // Пересчитываем интеграл и производную при изменении времени дискретизации
      float ratio = sampleTimeSec * 1000 / sampleTime;
      Ki *= ratio;
      Kd /= ratio;
      sampleTime = sampleTimeSec * 1000;
    }
    
    // Установка целевого значения
    void setTarget(float targetValue) {
      target = targetValue;
    }
    
    // Включение/выключение автоматического режима
    void setMode(bool automatic) {
      if (automatic && !inAuto) {
        // При переходе в автоматический режим сбрасываем интеграл
        integral = output;
        if (integral > outMax) integral = outMax;
        else if (integral < outMin) integral = outMin;
      }
      inAuto = automatic;
    }
    
    // Основная функция вычисления ПИД
    float compute(float currentInput) {
      if (!inAuto) return output;
      
      unsigned long now = millis();
      unsigned long timeChange = now - lastTime;
      
      // Проверяем, прошло ли достаточно времени
      if (timeChange < sampleTime) {
        return output;
      }
      
      input = currentInput;
      float error = target - input;
      
      // Пропорциональная составляющая
      float P = Kp * error;
      
      // Интегральная составляющая
      integral += Ki * error * (sampleTime / 1000.0);
      
      // Антивиндовп (ограничение интегральной составляющей)
      if (integral > outMax) integral = outMax;
      else if (integral < outMin) integral = outMin;
      
      float I = integral;
      
      // Дифференциальная составляющая
      float D = 0;
      if (derivativeOnInput) {
        // Производная по входу (уменьшает скачки при изменении уставки)
        D = -Kd * (input - prevInput) / (sampleTime / 1000.0);
      } else {
        // Производная по ошибке
        D = Kd * (error - prevError) / (sampleTime / 1000.0);
      }
      
      // Вычисляем выходное значение
      output = P + I + D;
      
      // Ограничиваем выходное значение
      if (output > outMax) output = outMax;
      else if (output < outMin) output = outMin;
      
      // Сохраняем значения для следующей итерации
      prevError = error;
      prevInput = input;
      lastTime = now;
      
      return output;
    }
    
    // Сброс регулятора
    void reset() {
      integral = 0;
      prevError = 0;
      prevInput = 0;
      output = 0;
    }
    
    // Получение текущих параметров
    float getKp() { return Kp; }
    float getKi() { return Ki; }
    float getKd() { return Kd; }
    float getTarget() { return target; }
    float getOutput() { return output; }
    float getInput() { return input; }
    
    // Установка режима производной
    void setDerivativeOnInput(bool onInput) {
      derivativeOnInput = onInput;
    }
};

#endif