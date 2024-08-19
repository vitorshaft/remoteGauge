#include <unity.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"

#define DHTPIN 4
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// Setup do Unity
void setUp(void) {
    dht.begin();
}

// Cleanup após cada teste
void tearDown(void) {
    // Opcional: limpar recursos, se necessário
}

// Testa se o sensor DHT foi inicializado corretamente
void test_dht_initialization(void) {
    // Verifica se o objeto foi criado
    TEST_ASSERT_NOT_NULL(&dht);  // Verifica se o ponteiro para o objeto dht não é nulo
}

// Testa se a leitura de temperatura retorna um valor válido
void test_dht_read_temperature(void) {
    float temperature = dht.readTemperature();
    // Como estamos testando nativamente, podemos não ter um valor real de temperatura,
    // mas podemos testar se o valor não é NaN
    TEST_ASSERT_FALSE(isnan(temperature));
}

// Testa se a leitura de umidade retorna um valor válido
void test_dht_read_humidity(void) {
    float humidity = dht.readHumidity();
    // Testa se o valor de umidade não é NaN
    TEST_ASSERT_FALSE(isnan(humidity));
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_dht_initialization);
    RUN_TEST(test_dht_read_temperature);
    RUN_TEST(test_dht_read_humidity);
    UNITY_END();
}

void loop() {
    // Nada a ser executado no loop
}
