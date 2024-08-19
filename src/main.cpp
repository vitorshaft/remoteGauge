#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DHTPIN 4     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);
Servo gauge;

float temperature = 0;
float humidity = 0;

// Funções das tarefas FreeRTOS
void readSensorTask(void *pvParameters);
void controlServoTask(void *pvParameters);

void setup() {
  Serial.begin(115200);
  Serial.println(F("DHTxx test with FreeRTOS!"));

  dht.begin();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  gauge.setPeriodHertz(50);
  gauge.attach(13, 500, 2400);

  // Criando as tarefas FreeRTOS
  xTaskCreate(readSensorTask, "Read Sensor", 2048, NULL, 1, NULL);
  xTaskCreate(controlServoTask, "Control Servo", 2048, NULL, 1, NULL);
}

void loop() {
  // O loop fica vazio porque o código agora é gerenciado por tarefas FreeRTOS
}

// Tarefa de leitura do sensor DHT11
void readSensorTask(void *pvParameters) {
  while (true) {
    // Aguarda 2 segundos entre as medições
    delay(2000);
    
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      continue; // Volta ao início do loop
    }

    float heatIndexC = dht.computeHeatIndex(temperature, humidity, false);
    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.print(F("%  Temperature: "));
    Serial.print(temperature);
    Serial.print(F("°C  Heat index: "));
    Serial.print(heatIndexC);
    Serial.println(F("°C"));
  }
}

// Tarefa de controle do servo motor
void controlServoTask(void *pvParameters) {
  while (true) {
    // Aguarda 2 segundos para sincronizar com a leitura do sensor
    delay(2000);

    int angle = map(temperature * 10, 200, 400, 0, 180);
    gauge.write(angle);
    
    Serial.print(F("Servo angle set to: "));
    Serial.println(angle);
  }
}
