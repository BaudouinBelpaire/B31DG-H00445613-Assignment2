#define PORT_DIGITAL_SIGNAL 9
#define A_PIN 1
#define LED_TASK7 7
#define ERROR_LED 10
#define PUSH_BUTTON 4
#define PWM_SIGNAL 5
#define INTERRUPT_PIN_TASK2 0
#define INTERRUPT_PIN_TASK3 2
#define CPU_PIN 8
#define TRUE 1
#define FALSE 0
#define STACK_SIZE_DEBUG 0

// Frequency Parameters: (Task 2, 3, 5)
const uint secondsToMicros = 1000000;
uint period;
const int timeout = 6;
unsigned long firstTimeTrig = 0;
volatile int flag;
const int trials = 2;

const int outputFreq = 800;
const int dutyCycle = 0.5*255 - 1;

struct FrequencyData {
    int frequencyTask2;
    int frequencyTask3;
};

FrequencyData myData;

//Time to toggle LED:
unsigned long eventSentTime;
unsigned long togglingTime;

const int CPU_frequency = 1800;

SemaphoreHandle_t frequencySemaphore;
QueueHandle_t eventQueue;

// Stack sizes:
const int stackT1 = 629;
const int stackT2 = 665;
const int stackT3 = 660;
const int stackT4 = 759;
const int stackT5 = 754;
const int stackT6 = 728;
const int stackT7 = 598;
const int stackT8 = 634;

// Task periods: 
const int periodT1 = 4;
const int periodT2 = 20;
const int periodT3 = 8;
const int periodT4 = 20;
const int periodT5 = 200;
const int periodT8 = 20;

// Task ID:
const int T1 = 1;
const int T2 = 2;
const int T3 = 3;
const int T4 = 4;
const int T5 = 5;
const int T6 = 6;
const int T7 = 7;
const int T8 = 8;

String stackStr;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PORT_DIGITAL_SIGNAL, OUTPUT);
  pinMode(LED_TASK7, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);
  pinMode(PUSH_BUTTON, INPUT_PULLDOWN);
  pinMode(INTERRUPT_PIN_TASK2, INPUT);
  pinMode(INTERRUPT_PIN_TASK3, INPUT);
  pinMode(PWM_SIGNAL, OUTPUT);
  analogWriteFrequency(outputFreq);
  analogWrite(PWM_SIGNAL, dutyCycle);

  frequencySemaphore = xSemaphoreCreateBinary();
  if(frequencySemaphore == NULL)
    {
        Serial.println("Frequency binary semaphore not created");
    }
    else
    {
        Serial.println("Frequency binary semaphore successfully created");
        xSemaphoreGive(frequencySemaphore);
    }
  
  eventQueue = xQueueCreate(5, sizeof(bool));

  xTaskCreate(task1, "task1", stackT1, NULL, 8, NULL);
  xTaskCreate(task2, "task2", stackT2, NULL, 6, NULL);
  xTaskCreate(task3, "task3", stackT3, NULL, 7, NULL);
  xTaskCreate(task4, "task4", stackT4, NULL, 6, NULL);
  xTaskCreate(task5, "task5", stackT5, NULL, 2, NULL);
  xTaskCreate(task6, "task6", stackT6, NULL, 1, NULL);
  xTaskCreate(task7, "task7", stackT7, NULL, 1, NULL);
  xTaskCreate(task8, "task8", stackT8, NULL, 6, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void ISR(){
  period = micros() - firstTimeTrig;
  firstTimeTrig = micros();
  flag++;
}

int trim(int value, int lower, int upper){
  int trimmedValue;
  if (value>upper){
    value = upper;
  }
  if (value<lower){
    value = lower;
  }
  trimmedValue = value;
  return trimmedValue;
}

int compute_frequency(){
  flag = 0;
  int frequency;
  unsigned long timeInit = millis();
  while(flag<trials && millis() - timeInit < timeout){}
  frequency = secondsToMicros/period;
  return frequency;
}

void CPU_work(int time){
  TickType_t tickPeriod = pdMS_TO_TICKS(time);
  TickType_t initialTickTime = xTaskGetTickCount();
  int counts = CPU_frequency * time / 1000;
  for (int i = 0; i<counts; i++){
    digitalWrite(CPU_PIN, LOW); // Dummy action to use the CPU
  }
}

void printStackUsage(int taskID, int stack){
  if (STACK_SIZE_DEBUG){
    stackStr = "Task " + String(taskID) + " requires a stack size of: " + String(stack);
    Serial.println(stackStr);
  }
}

void deadlineViolation(int taskID){
  String deadlineViolationStr = "Task " + String(taskID) + " violated the deadline!";
  Serial.println(deadlineViolationStr);
}

// Works
void task1(void *pvParameters){
  TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
  UBaseType_t stackHighWaterMark;
  int stackSize;
  unsigned long startTask;
  while(1){
    startTask = millis();
    digitalWrite(PORT_DIGITAL_SIGNAL, HIGH);
    delayMicroseconds(180);
    digitalWrite(PORT_DIGITAL_SIGNAL, LOW);
    delayMicroseconds(40);
    digitalWrite(PORT_DIGITAL_SIGNAL, HIGH);
    delayMicroseconds(530);
    digitalWrite(PORT_DIGITAL_SIGNAL, LOW);
    if(millis() - startTask > periodT1){
      deadlineViolation(T1);
    }
    stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
    stackSize = stackT1 - stackHighWaterMark;
    printStackUsage(T1, stackSize);
    vTaskDelay(pdMS_TO_TICKS(periodT1));
  }
}

// Works
void task2(void *pvParameters){
  TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
  UBaseType_t stackHighWaterMark;
  int stackSize;

  unsigned long startTask;
  unsigned long tmpStartTask;
  while(1){
    tmpStartTask = startTask;
    startTask = millis();
    //Serial.println(startTask - tmpStartTask);
    if(xSemaphoreTake(frequencySemaphore, pdMS_TO_TICKS(2))){
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_TASK2), ISR, RISING);
      myData.frequencyTask2 = compute_frequency();
      detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_TASK2));
      //Serial.println(myData.frequencyTask2);
      xSemaphoreGive(frequencySemaphore);
    }
    if(millis() - startTask > periodT2){
      deadlineViolation(T2);
    }
    stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
    stackSize = stackT2 - stackHighWaterMark;
    printStackUsage(T2, stackSize);
    vTaskDelay(pdMS_TO_TICKS(periodT2)); 
  }
}

// Works
void task3(void *pvParameters){
  TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
  UBaseType_t stackHighWaterMark;
  int stackSize;

  unsigned long startTask;
  unsigned long tmpStartTask;
  while(1){
    tmpStartTask = startTask;
    startTask = millis();
    //Serial.println(startTask-tmpStartTask);
    if(xSemaphoreTake(frequencySemaphore, pdMS_TO_TICKS(1))){
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_TASK3), ISR, RISING);
      myData.frequencyTask3 = compute_frequency();
      detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_TASK3));
      //Serial.println(myData.frequencyTask3);
      xSemaphoreGive(frequencySemaphore);
    }
    if(millis() - startTask > periodT3*1000){
      deadlineViolation(T3);
    }
    stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
    stackSize = stackT3 - stackHighWaterMark;
    printStackUsage(T3, stackSize);
    vTaskDelay(pdMS_TO_TICKS(periodT3)); 
  }
}

// Works
void task4(void *pvParameters){
  int analogueDataIn;
  int averageAnalogueIn;
  bool errorLedState;
  const int halfAnalogueRange = 4096/2 - 1;
  const int bufferSize = 10;
  int tmpBuffer[bufferSize] = {0};
  int buffer[bufferSize] = {0};

  TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
  UBaseType_t stackHighWaterMark;
  int stackSize;

  unsigned long startTask;
  while(1){
    startTask = millis();

    analogueDataIn = analogRead(A_PIN);
    buffer[0] = analogueDataIn;
    averageAnalogueIn = analogueDataIn;

    for (int i = 0; i<bufferSize-1; i++){
      buffer[i+1] = tmpBuffer[i];
      averageAnalogueIn += tmpBuffer[i];
      tmpBuffer[i] = buffer[i];
    }
    tmpBuffer[bufferSize] = buffer[bufferSize];
    averageAnalogueIn /= bufferSize;
    //Serial.println(averageAnalogueIn);

    if(averageAnalogueIn >= halfAnalogueRange){
      errorLedState = TRUE;
    }
    else{
      errorLedState = FALSE;
    }
    digitalWrite(ERROR_LED, errorLedState);
    if(millis() - startTask > periodT4){
      deadlineViolation(T4);
    }
    stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
    stackSize = stackT4 - stackHighWaterMark;
    printStackUsage(T4, stackSize);
    vTaskDelay(pdMS_TO_TICKS(periodT4));
  }
}

// Works
void task5(void *pvParameters){
  int trimmedFrequencyT2;
  int trimmedFrequencyT3;
  float frequencyT2;
  float frequencyT3;

  const int freqMinLimitT2 = 333;
  const int freqMaxLimitT2 = 1000;
  const int freqMinLimitT3 = 500;
  const int freqMaxLimitT3 = 1000;
  const int mappingMinLimit = 0;
  const int mappingMaxLimit = 9999;
  const float scalingFactor = 100.0;

  String unrangedFreq;
  String rangedFreq;

  TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
  UBaseType_t stackHighWaterMark;
  int stackSize;

  unsigned long startTask;
  while(1){
    startTask = millis();
    
    if(xSemaphoreTake(frequencySemaphore, pdMS_TO_TICKS(20))){
      trimmedFrequencyT2 = trim(myData.frequencyTask2, freqMinLimitT2, freqMaxLimitT2);
      trimmedFrequencyT3 = trim(myData.frequencyTask2, freqMinLimitT3, freqMaxLimitT3);
      frequencyT2 = map(trimmedFrequencyT2, freqMinLimitT2, freqMaxLimitT2, mappingMinLimit, mappingMaxLimit)/scalingFactor;
      frequencyT3 = map(trimmedFrequencyT3, freqMinLimitT3, freqMaxLimitT3, mappingMinLimit, mappingMaxLimit)/scalingFactor;
      unrangedFreq = String(myData.frequencyTask2) + ","  + String(myData.frequencyTask3);
      rangedFreq = String(frequencyT2) + ","  + String(frequencyT3);
      Serial.println(unrangedFreq);
      Serial.println(rangedFreq);
      //Serial.printf("%d,%d \n", myData.frequencyTask2, myData.frequencyTask3);
      //Serial.printf("%f,%f \n", frequencyT2, frequencyT3);
      xSemaphoreGive(frequencySemaphore);
    }

    if(millis() - startTask > periodT5){
      deadlineViolation(T5);
    }
    stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
    stackSize = stackT5 - stackHighWaterMark;
    printStackUsage(T5, stackSize);
    vTaskDelay(pdMS_TO_TICKS(periodT5));
  }
}

// Works
void task6(void *pvParameters){
  bool buttonReading;
  bool tmpButtonReading = LOW;
  bool event;

  TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
  UBaseType_t stackHighWaterMark;
  int stackSize;

  while(1){
    buttonReading = digitalRead(PUSH_BUTTON);

    if(buttonReading == HIGH && buttonReading != tmpButtonReading){
      eventSentTime = millis();
      event = TRUE;
      xQueueSend(eventQueue, &event, portMAX_DELAY);
    }

    tmpButtonReading = buttonReading;

    stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
    stackSize = stackT6 - stackHighWaterMark;
    printStackUsage(T6, stackSize);
  }
}

// Works
void task7(void *pvParameters){
  bool ledState = LOW;
  bool receivedEvent;

  String timeTakenStr;

  TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
  UBaseType_t stackHighWaterMark;
  int stackSize;

  while(1){
    xQueueReceive(eventQueue, &receivedEvent, portMAX_DELAY);
    if(receivedEvent == TRUE){
      ledState = !ledState;
      digitalWrite(LED_TASK7, ledState);
      togglingTime = millis() - eventSentTime;
      timeTakenStr = "It took " + String(togglingTime) + " ms to toggle the LED";
      Serial.println(timeTakenStr);
    }

    stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
    stackSize = stackT7 - stackHighWaterMark;
    printStackUsage(T7, stackSize);
  }
}

// Works
void task8(void *pvParameters){
  int time = 2;

  TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
  UBaseType_t stackHighWaterMark;
  int stackSize;

  unsigned long startTask;
  while(1){
    startTask = millis();
    CPU_work(time);
    if(millis() - startTask > periodT8){
      deadlineViolation(T8);
    }
    stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
    stackSize = stackT8 - stackHighWaterMark;
    printStackUsage(T8, stackSize);
    vTaskDelay(pdMS_TO_TICKS(periodT8));
  }
}