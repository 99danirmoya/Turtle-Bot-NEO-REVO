/* ***********************************************************************************************************************************************************
TURTLE BOT NEO REVO: este skecth sirve para programar un robot de dos ruedas que cuenta con un sensor climatico BME280, un GPS NEO-6M, un magnetometro
HMC5883L y un sensor de PM SDS011 que publica las medidas cada 5 segundos en el servidor MQTT https://broker.hivemq.com para ser procesadas en NodeRED.
Además, en NodeRED se crean los topicos necesarios para controlar la pantalla OLED, el LED, los servos de rotacion continua y el zumbador a bordo del robot,
que reciben los parametros de un dashboard de NodeRED por medio de estar suscrito a los topicos donde se envian dichos parametros.
*********************************************************************************************************************************************************** */

// ===========================================================================================================================================================
// INLCUSION DE LIBRERIAS
// ===========================================================================================================================================================
#include <Arduino.h>                                                                                                     // Libreria de Arduino para garantizar que se aplican funciones propias tambien en la ESP32 S3

#include <WiFiManager.h>                                                                                                 // Liberia para crear un hotspot desde el que configurar la conexion WiFi
#include <WiFiClientSecure.h>                                                                                            // Liberia para añadir certificados de conexion segura
#include <PubSubClient.h>                                                                                                // Libreria para MQTT
#include <ArduinoJson.h>                                                                                                 // Libreria para parsear textos en formato JSON facilmente

#include <Wire.h>                                                                                                        // Libreria del bus I2C

#include <Adafruit_Sensor.h>                                                                                             // Libreria para usar sensores de Adafruit
#include <Adafruit_BME280.h>                                                                                             // Libreria del BME280

#include <Adafruit_GFX.h>                                                                                                // Libreria complementaria para el panel OLED
#include <Adafruit_SSD1306.h>                                                                                            // Libreria especifica para el panel OLED de 128x64 pixeles

#include <ESP32Servo.h>                                                                                                  // Libreria para manejos de servos para placas que usen ESP32

#include <TinyGPS++.h>                                                                                                   // Libreria para usar el GPS NEO-6M por puerto serie

#include <Adafruit_HMC5883_U.h>                                                                                          // Libreria para usar la brujula HMC5883L

#include <SoftwareSerial.h>                                                                                              // Libreria para crear puertos serie virtuales en GPIO comunes
// LIBRERIAS END =============================================================================================================================================

// ===========================================================================================================================================================
// MACROS (de ser necesarias)
// ===========================================================================================================================================================
// Macro serial monitor
#define ENABLE_DEBUG 1                                                                                                   // Boolean to enable/disable serial monitor

// Macros OLED I2C -------------------------------------------------------------------------------------------------------------------------------------------
#define OLED_ADDR 0x3C                                                                                                   // Direccion I2C
#define SCREEN_WIDTH 128                                                                                                 // Pixeles de ancho
#define SCREEN_HEIGHT 64                                                                                                 // Pixeles de alto
#define OLED_RESET -1                                                                                                    // Reset del OLED
#define SDA_OLED 18                                                                                                      // Se asignan los pines I2C (SDA, SCL) del panel OLED. ESTA CONEXION SDA, SCL NO ES ACCESIBLE DESDE EL PINOUT, ES UNA CONEXION INTERNA
#define SCL_OLED 17

// Macros GPS ------------------------------------------------------------------------------------------------------------------------------------------------
#define SerialGPS Serial1                                                                                                // Se crea un nuevo puerto serie para el GPS (el predeterminado, 'Serial', se usa para el monitor serie)
#define GPS_RX_PIN 44                                                                                                    // Pin RX del ESP32 al pin TX del GPS
#define GPS_TX_PIN 43                                                                                                    // Pin TX del ESP32 al pin RX del GPS
#define GPS_BAUD_RATE 9600                                                                                               // Baudios para la comunicacion serie del GPS

// Macros SDS011 ---------------------------------------------------------------------------------------------------------------------------------------------
#define SDS_RX 47                                                                                                        // Pin RX virtual del ESP32 al pin TX del SDS011
#define SDS_TX 48                                                                                                        // Pin TX virtual del ESP32 al pin RX del SDS011

// Macros servos ---------------------------------------------------------------------------------------------------------------------------------------------
#define PIN_SERVO_RIGHT 45                                                                                               // Se asignan los pines a los que se conectan los cables 'DATA' de cada servo
#define PIN_SERVO_LEFT 41

// Macros BME280 I2C -----------------------------------------------------------------------------------------------------------------------------------------
#define SDA_BME 38                                                                                                       // Se asignan los pines I2C (SDA, SCL) del BME280. ESTOS SI SON ACCESIBLES DESDE EL PINOUT Y SE PODRIA USAR REALMENTE COMO UN BUS I2C AL QUE CONECTARLE VARIOS DISPOSITIVOS
#define SCL_BME 39
#define SEALEVELPRESSURE_HPA (1015)                                                                                      // Referencia de la presion atmosferica al nivel del mar

// Macros luz LED --------------------------------------------------------------------------------------------------------------------------------------------
#define LED_PIN 40
#define FREQUENCY 5000                                                                                                   // Frecuencia de la señal PWM
#define RESOLUTION 8                                                                                                     // Resolucion del PWM

// Macros medidor del voltaje de la bateria ------------------------------------------------------------------------------------------------------------------
#define VBAT_PIN 1                                                                                                       // Pin internamente conectado a un circuito que lee el voltaje de la bateria en el conector inferior de la LilyGO

// Macros buzzer ---------------------------------------------------------------------------------------------------------------------------------------------
#define BUZZER_PIN 42

// Macros para temporalizaciones -----------------------------------------------------------------------------------------------------------------------------
// MACROS END ================================================================================================================================================

// ===========================================================================================================================================================
// CONSTRUCTORES DE OBJETOS DE CLASE DE LIBRERIA, VARIABLES GLOBALES, CONSTANTES...
// ===========================================================================================================================================================
WiFiClientSecure secureClient;                                                                                           // Objeto de la libreria WiFiManager
PubSubClient mqttClient(secureClient);                                                                                   // Objeto de la libreria MQTT

TinyGPSPlus gps;                                                                                                         // Objeto del GPS

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

SoftwareSerial sds(SDS_RX, SDS_TX);                                                                                      // Inicializacion del puerto serie virtual para el sensor SDS011

Adafruit_BME280 bme;                                                                                                     // Objeto del sensor BME280

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);                                                  // Objeto del OLED

Servo miServoRight;                                                                                                      // Objeto del servo derecho
Servo miServoLeft;                                                                                                       // Objeto del servo izquierdo
// CONSTRUCTORES END =========================================================================================================================================

// ===========================================================================================================================================================
// STRUCTS
// ===========================================================================================================================================================
// Struct para los datos de sensores -------------------------------------------------------------------------------------------------------------------------
struct sensorData{
  float temp, hum, alt, pres, vbat, lat, lon, veloc, pm2_5, pm10, heading;                                               // Definicion de las variables 'float'
  uint8_t sat;                                                                                                           // Variable en la que se guarda el numero de satelites en cobertura del GPS
};

// Struct para los valores de los ejes del joystick virtual --------------------------------------------------------------------------------------------------
struct joystickData{
  int8_t potXaxis = 0, potYaxis = 0;
};
// STRUCTS END ===============================================================================================================================================

// ===========================================================================================================================================================
// GLOBAL VARIABLES
// ===========================================================================================================================================================
const char* mqtt_server = "srv-iot.diatel.upm.es";                                                                       // Broker MQTT de la UPM
const int mqtt_port = 8883;                                                                                              // Puerto del Broker MQTT
const char* mqttTopicPub = "v1/devices/me/telemetry";
const char* mqttTopicSub = "v1/devices/me/attributes";
const char* access_token = "c80zn9tfv9oiluyp2tss";                                                                       // Token unico del dispositivo en Thingsboard

const char* root_ca = R"EOF(-----BEGIN CERTIFICATE-----
MIIF3jCCA8agAwIBAgIQAf1tMPyjylGoG7xkDjUDLTANBgkqhkiG9w0BAQwFADCB
iDELMAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0pl
cnNleSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNV
BAMTJVVTRVJUcnVzdCBSU0EgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTAw
MjAxMDAwMDAwWhcNMzgwMTE4MjM1OTU5WjCBiDELMAkGA1UEBhMCVVMxEzARBgNV
BAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNleSBDaXR5MR4wHAYDVQQKExVU
aGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMTJVVTRVJUcnVzdCBSU0EgQ2Vy
dGlmaWNhdGlvbiBBdXRob3JpdHkwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAwggIK
AoICAQCAEmUXNg7D2wiz0KxXDXbtzSfTTK1Qg2HiqiBNCS1kCdzOiZ/MPans9s/B
3PHTsdZ7NygRK0faOca8Ohm0X6a9fZ2jY0K2dvKpOyuR+OJv0OwWIJAJPuLodMkY
tJHUYmTbf6MG8YgYapAiPLz+E/CHFHv25B+O1ORRxhFnRghRy4YUVD+8M/5+bJz/
Fp0YvVGONaanZshyZ9shZrHUm3gDwFA66Mzw3LyeTP6vBZY1H1dat//O+T23LLb2
VN3I5xI6Ta5MirdcmrS3ID3KfyI0rn47aGYBROcBTkZTmzNg95S+UzeQc0PzMsNT
79uq/nROacdrjGCT3sTHDN/hMq7MkztReJVni+49Vv4M0GkPGw/zJSZrM233bkf6
c0Plfg6lZrEpfDKEY1WJxA3Bk1QwGROs0303p+tdOmw1XNtB1xLaqUkL39iAigmT
Yo61Zs8liM2EuLE/pDkP2QKe6xJMlXzzawWpXhaDzLhn4ugTncxbgtNMs+1b/97l
c6wjOy0AvzVVdAlJ2ElYGn+SNuZRkg7zJn0cTRe8yexDJtC/QV9AqURE9JnnV4ee
UB9XVKg+/XRjL7FQZQnmWEIuQxpMtPAlR1n6BB6T1CZGSlCBst6+eLf8ZxXhyVeE
Hg9j1uliutZfVS7qXMYoCAQlObgOK6nyTJccBz8NUvXt7y+CDwIDAQABo0IwQDAd
BgNVHQ4EFgQUU3m/WqorSs9UgOHYm8Cd8rIDZsswDgYDVR0PAQH/BAQDAgEGMA8G
A1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQEMBQADggIBAFzUfA3P9wF9QZllDHPF
Up/L+M+ZBn8b2kMVn54CVVeWFPFSPCeHlCjtHzoBN6J2/FNQwISbxmtOuowhT6KO
VWKR82kV2LyI48SqC/3vqOlLVSoGIG1VeCkZ7l8wXEskEVX/JJpuXior7gtNn3/3
ATiUFJVDBwn7YKnuHKsSjKCaXqeYalltiz8I+8jRRa8YFWSQEg9zKC7F4iRO/Fjs
8PRF/iKz6y+O0tlFYQXBl2+odnKPi4w2r78NBc5xjeambx9spnFixdjQg3IM8WcR
iQycE0xyNN+81XHfqnHd4blsjDwSXWXavVcStkNr/+XeTWYRUc+ZruwXtuhxkYze
Sf7dNXGiFSeUHM9h4ya7b6NnJSFd5t0dCy5oGzuCr+yDZ4XUmFF0sbmZgIn/f3gZ
XHlKYC6SQK5MNyosycdiyA5d9zZbyuAlJQG03RoHnHcAP9Dc1ew91Pq7P8yF1m9/
qS3fuQL39ZeatTXaw2ewh0qpKJ4jjv9cJ2vhsE/zB+4ALtRZh8tSQZXq9EfX7mRB
VXyNWQKV3WKdwrnuWih0hKWbt5DHDAff9Yk2dDLWKMGwsAvgnEzDHNb842m1R0aB
L6KCq9NjRHDEjf8tM7qtj3u1cIiuPhnPQCjY/MiQu12ZIvVS5ljFH4gxQ+6IHdfG
jjxDah2nGN59PRbxYvnKkKj9
-----END CERTIFICATE-----)EOF";                                                                                          // Certificado para el cifrado TLS de MQTT en Thingsboard

volatile bool oledStateON = false, oledStateOFF = false, ledStateON = false, ledStateOFF = false, claxonState = false;   // Volatile flags that change the state among tasks
// GLOBAL VARIABLES END ======================================================================================================================================

// ===========================================================================================================================================================
// FREERTOS CONSTRUCTORS
// ===========================================================================================================================================================
// Task handles ----------------------------------------------------------------------------------------------------------------------------------------------
TaskHandle_t mqttTaskHandle, sensorTaskHandle, actuatorTaskHandle, flagsTaskHandle;

// Sensor queue ----------------------------------------------------------------------------------------------------------------------------------------------
QueueHandle_t sensorQueuePub = 0, sensorQueueOLED = 0, joystickQueue = 0;

// Semaphore -------------------------------------------------------------------------------------------------------------------------------------------------
SemaphoreHandle_t semaphoreSerial;
// FREERTOS CONSTRUCTORS END =================================================================================================================================

// ===========================================================================================================================================================
// FUNCTION PROTOTYPES
// ===========================================================================================================================================================
// Tasks -----------------------------------------------------------------------------------------------------------------------------------------------------
void mqttTask(void*);
void sensorTask(void*);
void flagsTask(void*);
void actuatorTask(void*);

// Auxiliary functions ---------------------------------------------------------------------------------------------------------------------------------------
void callback(char*, byte*, unsigned int);
void reconnect();
float current_heading();
// ===========================================================================================================================================================

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// MQTT TASK - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO PARA EL MQTT PARA EL PUBLISHING
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void mqttTask(void* pvParameters) {
  TickType_t lastReceivedTime = xTaskGetTickCount();                                                                     // Timestamp for the last data received
  const TickType_t errorThreshold = pdMS_TO_TICKS(6000);                                                                 // 6-second threshold

  while(true){
    if(!mqttClient.connected()){                                                                                         // Si no hay conexion
      reconnect();                                                                                                       // Entra la funcion de reconexion
    }
    mqttClient.loop();

    sensorData data;
    if(!xQueueReceive(sensorQueuePub, &data, pdMS_TO_TICKS(100))){
      if((xTaskGetTickCount() - lastReceivedTime) > errorThreshold){
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("Error recibiendo datos de la cola de sensores!"));
          xSemaphoreGive(semaphoreSerial);
        }
        lastReceivedTime = xTaskGetTickCount();                                                                          // Reset the timestamp to avoid repeated prints
      }
    }else{
      lastReceivedTime = xTaskGetTickCount();                                                                            // Update the timestamp when data is received
      // ===================================================================================================================================================
      // MQTT PUBLISH
      // ===================================================================================================================================================
      if(WiFi.status() == WL_CONNECTED){                                                                                 // Check WiFi connection status
        char dataStr[256];                                                                                               // Se crea un string de caracteres para guardar ambas medidas, se reservan 60 espacios
        sprintf(dataStr, "{\"temperature\":%5.2f,\"pressure\":%6.2f,\"altitude\":%4.2f,\"humidity\":%4.2f,\"battery\":%4.3f,\"latitude\":%10.6f,\"longitude\":%10.6f,\"sats\":%1d,\"speed\":%5.2f,\"PM2.5\":%6.2f,\"PM10\":%6.2f,\"heading\":%5.2f}",
                data.temp, data.pres, data.alt, data.hum, data.vbat, data.lat, data.lon, data.sat, data.veloc, data.pm2_5, data.pm10, data.heading);  // La funcion 'sprintf' de C++ se usa para poder introducir las medidas y elegir el ancho de numero y su precision. Las medidas se separan con una coma ','
        
        if(mqttClient.publish(mqttTopicPub, dataStr)){                                                                   // Se publica el string con los datos de los sensores en el topico 'moya/sensores'
          Serial.println(dataStr);                                                                                       // Muestra en el serial de Arduino el string
        }else{
          if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
            Serial.println(F("Failed to publish data"));
            xSemaphoreGive(semaphoreSerial);
          }
        }
        // MQTT PUBLISH END ==================================================================================================================================
      }else{
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("WiFi Disconnected"));
          xSemaphoreGive(semaphoreSerial);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
// MQTT TASK END ---------------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// SENSOR TASK - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO PARA LEER LOS SENSORES
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void sensorTask(void* pvParameters){
  int pm25int, pm10int;
  float pm25last = 0.0, pm10last = 0.0;

  while(true){
    sensorData data;

    data.temp = bme.readTemperature();                                                                                   // Variable que guarda la temperatura del sensor BME280
    data.pres = bme.readPressure() / 100.0F;                                                                             // Variable que guarda la presion del sensor BME280
    data.alt = bme.readAltitude(SEALEVELPRESSURE_HPA);                                                                   // Variable que guarda la altitud del sensor BME280
    data.hum = bme.readHumidity();                                                                                       // Variable que guarda la humedad del sensor BME280
    data.vbat = (float)(analogRead(VBAT_PIN)) / 4095 * 2 * 3.3 * 1.1;                                                    // Variable que guarda el valor del voltaje en el conector de batería
    data.heading = current_heading();                                                                                    // Variable que guarda la orientacion del robot con respecto a N

    // Preparacion del GPS para tener cobertura satelite y poder enviar coordenadas --------------------------------------------------------------------------
    while(SerialGPS.available() > 0){                                                                                    // Mientras haya bytes disponibles en el puerto serie del GPS
      if(gps.encode(SerialGPS.read())){                                                                                  // Se confirma que haya lecturas del GPS
        if (gps.location.isValid()) {                                                                                    // Si las coordenadas recogidas son validas
          data.lat = gps.location.lat();                                                                                 // Se guardan en las variables 'lat' y 'lon' dichas coordenadas
          data.lon = gps.location.lng();
          data.sat = gps.satellites.value();
          data.veloc = gps.speed.kmph();
        }else{                                                                                                           // En caso de que sean invalidas, se muestra un mensaje en el monitor serie
          data.lat = 0.0;                                                                                                // Por marcar un numero definido, la ubicacion se fija a 0 en ambos 'lat' y 'lon'
          data.lon = 0.0;
          data.sat = 0;
          data.veloc = 0.0;
        }
      }
    }
    if(data.sat == 0){
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("\tINVALID GPS "));
        xSemaphoreGive(semaphoreSerial);
      }
    }

    // Preparacion del SDS011 para mandar los bytes que contienen las medidas de PM2.5 y PM10 ----------------------------------------------------------------
    while(sds.available() && sds.read() != 0xAA);                                                                        // Look for the starting byte of the SDS011 data frame
    if(sds.available()){
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("\tData available from SDS011..."));
        xSemaphoreGive(semaphoreSerial);
      }
    }
    byte buffer[10];                                                                                                     // Once we have the starting byte, attempt to read the next 9 bytes
    buffer[0] = 0xAA;                                                                                                    // The starting byte we already found
    if(sds.available() >= 9){
      sds.readBytes(&buffer[1], 9);
      if(buffer[9] == 0xAB){                                                                                             // Check if the last byte is the correct ending byte
        pm25int = (buffer[3] << 8) | buffer[2];
        pm10int = (buffer[5] << 8) | buffer[4];
        data.pm2_5 = pm25int / 10.0;
        data.pm10 = pm10int / 10.0;

        pm25last = data.pm2_5;
        pm10last = data.pm10;
      }else{
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\tInvalid ending byte from SDS011"));
          xSemaphoreGive(semaphoreSerial);
        }
        data.pm2_5 = pm25last;
        data.pm10 = pm10last;
      }
    } else {
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("\tNot enough data from SDS011"));
        xSemaphoreGive(semaphoreSerial);
      }
    }

    if((xTaskGetTickCount() > pdMS_TO_TICKS(15000)) && (gps.charsProcessed() < 10)){                                     // Si durante 15 segundos se cumple que los caracteres procesados del GPS son menos de 10, implica que hay un error leyendo el GPS, se reporta el fallo y se bloquea el programa hasta que se solvente el error
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("\tGPS no detectado: comprueba el cableado"));
        xSemaphoreGive(semaphoreSerial);
      }
      vTaskDelay(pdMS_TO_TICKS(15000));
    }
    // -------------------------------------------------------------------------------------------------------------------------------------------------------

    if(!xQueueSend(sensorQueuePub, &data, pdMS_TO_TICKS(100))){
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("\tCola de sensores publisher ha fallado!"));
        xSemaphoreGive(semaphoreSerial);
      }
    }
    if(!xQueueSend(sensorQueueOLED, &data, pdMS_TO_TICKS(100))){
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("\tCola de sensores OLED ha fallado!"));
        xSemaphoreGive(semaphoreSerial);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5000));                                                                                     // Read every 5 seconds
  }
}
// SENSOR TASK END -------------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FLAGS TASK - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO QUE GESTIONA EL I/O DIGITAL CON VOLATILE BOOL FLAGS
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void flagsTask(void* pvParameters){
  TickType_t lastReceivedTime = xTaskGetTickCount();                                                                     // Timestamp for the last data received
  const TickType_t errorThreshold = pdMS_TO_TICKS(6000);                                                                 // 6-second threshold
  
  TickType_t claxonStartTime = 0;
  const TickType_t claxonTimeON = pdMS_TO_TICKS(500);
  bool claxonON = false;

  while(true){
    // ON LED ------------------------------------------------------------------------------------------------------------------------------------------------
    if(ledStateON){
      for(int i = 0; i <= 255; i += 1){
        ledcWrite(LED_PIN, i);
        vTaskDelay(pdMS_TO_TICKS(1));
      }
      ledStateON = false;
    }

    // OFF LED -----------------------------------------------------------------------------------------------------------------------------------------------
    if(ledStateOFF){
      for(int i = 255; i >= 0; i -= 1){
        ledcWrite(LED_PIN, i);
        vTaskDelay(pdMS_TO_TICKS(1));
      }
      ledStateOFF = false;
    }

    // ON OLED -----------------------------------------------------------------------------------------------------------------------------------------------
    sensorData data;

    if(oledStateON){                                                                                                     // Actualiza el OLED si esta encendido y han pasado 5 segundos
      if(!xQueueReceive(sensorQueueOLED, &data, pdMS_TO_TICKS(100))){
        if((xTaskGetTickCount() - lastReceivedTime) > errorThreshold){
          if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
            Serial.println(F("\t\t\tError recibiendo datos de la cola de sensores"));
            xSemaphoreGive(semaphoreSerial);
          }
          lastReceivedTime = xTaskGetTickCount();
        }
      } else {
        lastReceivedTime = xTaskGetTickCount();
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){  
          Serial.println(F("\t\t\tACTUALIZAR OLED"));
          xSemaphoreGive(semaphoreSerial);
        }
        oled.clearDisplay();                                                                                             // Se limpia el buffer del OLED
        oled.setTextSize(1);                                                                                             // Se selecciona el tamaño de la letra

        oled.setCursor(10, 5);                                                                                           // Se indica el pixel donde se quiere escribir (X,Y)
        oled.print("Temp: ");                                                                                            // Se escribe un string
        oled.setCursor(40, 5);
        oled.print(data.temp);                                                                                           // Se escribe el valor actual de la temperatura del BME280
        oled.setCursor(75, 5);
        oled.print((char)247);
        oled.setCursor(81, 5);
        oled.println("C");

        oled.setCursor(10, 17);
        oled.print("Pres: ");
        oled.setCursor(40, 17);
        oled.print(data.pres);
        oled.setCursor(80, 17);
        oled.print(" hPa");

        oled.setCursor(10, 29);
        oled.print("Alti: ");
        oled.setCursor(40, 29);
        oled.print(data.alt);
        oled.setCursor(70, 29);
        oled.print(" m");

        oled.setCursor(10, 41);
        oled.print("Hume: ");
        oled.setCursor(40, 41);
        oled.print(data.hum);
        oled.setCursor(70, 41);
        oled.print(" %");

        oled.setCursor(10, 53);
        oled.print("Bate: ");
        oled.setCursor(40, 53);
        oled.print(data.vbat);                                                                                           // Calculo matematico siguiendo el circuito de lectura de bateria de LilyGO para expresar el voltaje en el conector de bateria
        oled.setCursor(65, 53);
        oled.print(" V");

        oled.display();
      }
    }

    // OFF OLED ---------------------------------------------------------------------------------------------------------------------------------------------
    if(oledStateOFF){
      oledStateON = false;
      oled.clearDisplay();                                                                                               // Se limpia el buffer del OLED
      oled.display();                                                                                                    // Se printea el buffer que, como no es nada, se apaga
      oledStateOFF = false;
    }

    // CLAXON FOR 500 ms -------------------------------------------------------------------------------------------------------------------------------------
    if(claxonState && !claxonON){
      ledcWriteTone(BUZZER_PIN, 500);
      claxonStartTime = xTaskGetTickCount();
      claxonON = true;
      claxonState = false;
    }
    if(claxonON && (xTaskGetTickCount() - claxonStartTime >= claxonTimeON)){
      ledcWriteTone(BUZZER_PIN, 0);
      claxonON = false;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));                                                                                     // Small delay
  }
}
// FLAGS TASK END --------------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// ACTUATOR TASK - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO PARA EJECUTAR LAS ORDENES DE LOS SERVOS
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void actuatorTask(void* pvParameters){
  int8_t servoXPosition, servoYPosition;

  while (true) {
    joystickData joyData;

    if(!xQueueReceive(joystickQueue, &joyData, portMAX_DELAY)){
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("\t\t\tError recibiendo datos de la cola del joystick"));
        xSemaphoreGive(semaphoreSerial);
      }
    }else{
      servoXPosition = map(joyData.potXaxis, -100, 100, 45, -45);                                                        // Posición neutra del eje X del joystick es 0, que corresponde a 0º en el servo. Respectivamente, -100 es 45º y 100, -45º. Devuelve valores de -45 a 45
      servoYPosition = map(joyData.potYaxis, -100, 100, -45, 45);                                                        // Lo mismo para el eje Y del joystick es al revés, -100 es -45º y 100, 45º

      miServoRight.write(90 + servoYPosition + servoXPosition);                                                          // 90 + 45 + 0 = 135 -> Hacia adelante
      miServoLeft.write(90 - servoYPosition + servoXPosition);                                                           // 90 - 45 + 0 = 45  -> Hacia atras PORQUE ESTA PUESTO "ESPEJO" RESPECTO DEL OTRO EN EL ROBOT; CON ELLO SE MUEVE HACIA ADELANTE

      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.print(F("\t\t\tEje X: ")); Serial.print(joyData.potXaxis); Serial.print(F("\tEje Y: ")); Serial.println(joyData.potYaxis);
        Serial.print(F("\t\t\tServo X: ")); Serial.print(servoXPosition); Serial.print(F("\tServo Y: ")); Serial.println(servoYPosition);
        xSemaphoreGive(semaphoreSerial);
      }

      /*| -------------------------------------------------------------------------------------------------------- |
        |                                           Tabla de movimientos                                           |
        | -------------------------------------------------------------------------------------------------------- |
        | potYaxis  | potXaxis  | servoYPosition | servoXPosition | leftServo.write | rightServo.write | RESULTADO |
        | --------- | --------- | -------------- | -------------- | --------------- | ---------------- | --------- |
        |    100    |     0     |       45       |       0        |       45        |       135        | ADELANTE  | 
        |   -100    |     0     |      -45       |       0        |      135        |        45        |   ATRÁS   |
        |     0     |   -100    |       0        |       45       |      135        |       135        | IZQUIERDA |
        |     0     |    100    |       0        |      -45       |       45        |        45        |  DERECHA  |
        | --------- | --------- | -------------- | -------------- | --------------- | ---------------- | --------- |

        Teniendo en cuenta que el punto neutro es ambos a 90, ambos a 45 no significa moverse hacia adelante y, ambos a 135, tampoco hacia
        atrás. Por el contrario, estos dos casos representarían, respectivamente, el giro a derechas y el giro a izquierdas.
        El hecho de que, por ejemplo, hacia adelante sea LEFT a 45 y RIGHT a 135 es porque los motores están cambiados de sentido cuando
        se montan en el chasis                                                                                                        */
    }
    vTaskDelay(pdMS_TO_TICKS(750));                                                                                      // Small delay
  }
}
// ACTUATOR TASK END -----------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION SETUP - SOLO SE EJECUTA UNA VEZ
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void setup(){
  #if ENABLE_DEBUG == 1                                                                                                  // Activar o desactivar desde "configuration.h" el monitor serial para debugging
    Serial.begin(115200);
  #endif
  Serial.println(F("============== SETUP =============="));

  // =======================================================================================================================================================
  // INICIALIZACION DE I/O
  // =======================================================================================================================================================
  // Buzzer ------------------------------------------------------------------------------------------------------------------------------------------------
  ledcAttach(BUZZER_PIN, FREQUENCY, RESOLUTION);                                                                         // Inicializacion del PWM para el buzzer

  // LED ---------------------------------------------------------------------------------------------------------------------------------------------------
  ledcAttach(LED_PIN, FREQUENCY, RESOLUTION);                                                                            // Inicializacion del PWM para el LED

  // Servos ------------------------------------------------------------------------------------------------------------------------------------------------
  miServoRight.attach(PIN_SERVO_RIGHT);                                                                                  // Union del servo al pin correspondiente
  miServoLeft.attach(PIN_SERVO_LEFT);
  miServoRight.write(90);
  miServoLeft.write(90);

  // GPS ---------------------------------------------------------------------------------------------------------------------------------------------------
  SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);                                                    // Inicializacion de la comunicacion por serial para el GPS
  Serial.print(F("Testing TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());

  // SDS011 ------------------------------------------------------------------------------------------------------------------------------------------------
  sds.begin(9600);

  // OLED --------------------------------------------------------------------------------------------------------------------------------------------------
  Wire1.begin(SDA_OLED, SCL_OLED);                                                                                       // Initialize I2C for the OLED. SDA, SCL
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.display();                                                                                                        // Display initialization - LOGO ADAFRUIT
  vTaskDelay(pdMS_TO_TICKS(1000));
  oled.clearDisplay();                                                                                                   // Clear the display
  oled.setTextSize(3);
  oled.setTextColor(SSD1306_WHITE);
  oled.display();

  // BME280 ------------------------------------------------------------------------------------------------------------------------------------------------
  Wire.begin(SDA_BME, SCL_BME);                                                                                          // Initialize I2C for the BME280. SDA, SCL
  if(!bme.begin(0x76)){
    Serial.println(F("No encuentro un sensor BME280 valido!"));
    while(1);
  }else{
    Serial.println(F("BME280 inicializado!"));
  }

  // HMC5883 -----------------------------------------------------------------------------------------------------------------------------------------------
  if(!mag.begin()){
    Serial.println(F("No encuentro un sensor HMC5883L valido!"));
    while(1);
  }else{
    Serial.println(F("HMC5883L inicializada!"));
  }
  // INICIALIZACION DE I/O END =============================================================================================================================

  // -------------------------------------------------------------------------------------------------------------------------------------------------------
  // CONFIGURACION DEL HOTSPOT QUE CREA EL ESP. 3 MODOS: NOMBRE AUTOMATICO CON ID DEL ESP, NOMBRE A ELEGIR Y NOMBRE A ELEGIR CON CONTRASEÑA A ELEGIR
  // -------------------------------------------------------------------------------------------------------------------------------------------------------
  WiFiManager wm;                                                                                                        // WiFiManager inicializacion local

  bool res;                                                                                                              // Variable de la libreria WiFiManager para comprobar el estado de conexion a la red WiFi

  res = wm.autoConnect("AP_Moya", "m3di4l4b");                                                                           // Modo del hotspot con nombre y contraseña personalizados

  if(!res){
    Serial.println(F("Incapaz de conectarse a la WiFi elegida, RESET"));
    delay(1000);
    ESP.restart();
  }else{                                                                                                                 // Si 'res' devuelve un true, es que la conexion es exitosa
    Serial.println(F("¡Conectado a la WiFi elegida!"));
    Serial.print(F("IP Address: "));
    Serial.println(WiFi.localIP());
  }
  // CONFIGURACION DEL HOTSPOT END -------------------------------------------------------------------------------------------------------------------------

  // Inicializacion de la conexion cifrada
  secureClient.setCACert(root_ca);

  // Initialize MQTT broker and callback function
  mqttClient.setServer(mqtt_server, mqtt_port);                                                                          // Funcion de la libreria PubSubClient para establecer el servidor MQTT y su puerto
  mqttClient.setCallback(callback);                                                                                      // Funcion de la liberia PubSubClient para definir la funcion de los callbacks

  // =======================================================================================================================================================
  // INICIALIZACION DE freeRTOS
  // =======================================================================================================================================================
  // Create the queues
  sensorQueuePub = xQueueCreate(1, sizeof(sensorData));                                                                  // Queue can hold up to 12 sensor readings
  sensorQueueOLED = xQueueCreate(1, sizeof(sensorData));
  joystickQueue = xQueueCreate(1, sizeof(joystickData));

  // Create the semaphores
  semaphoreSerial = xSemaphoreCreateMutex();

  // Create tasks and assign them to different cores
  xTaskCreatePinnedToCore(
    mqttTask,                                                                                                            /* Function to implement the task */
    "mqttTask",                                                                                                          /* Name of the task */
    10000,                                                                                                               /* Stack size in bytes */
    NULL,                                                                                                                /* Task input parameter */
    1,                                                                                                                   /* Priority of the task */
    &mqttTaskHandle,                                                                                                     /* Task handle. */
    1                                                                                                                    /* Core where the task should run */
  );

  xTaskCreatePinnedToCore(
    sensorTask,                                                                                                          /* Function to implement the task */
    "sensorTask",                                                                                                        /* Name of the task */
    5000,                                                                                                                /* Stack size in bytes */
    NULL,                                                                                                                /* Task input parameter */
    1,                                                                                                                   /* Priority of the task */
    &sensorTaskHandle,                                                                                                   /* Task handle. */
    0                                                                                                                    /* Core where the task should run */
  );

  xTaskCreatePinnedToCore(
    actuatorTask,                                                                                                        /* Function to implement the task */
    "actuatorTask",                                                                                                      /* Name of the task */
    5000,                                                                                                                /* Stack size in bytes */
    NULL,                                                                                                                /* Task input parameter */
    1,                                                                                                                   /* Priority of the task */
    &actuatorTaskHandle,                                                                                                 /* Task handle. */
    0                                                                                                                    /* Core where the task should run */
  );

  xTaskCreatePinnedToCore(
    flagsTask,                                                                                                           /* Function to implement the task */
    "flagsTask",                                                                                                         /* Name of the task */
    5000,                                                                                                                /* Stack size in bytes */
    NULL,                                                                                                                /* Task input parameter */
    1,                                                                                                                   /* Priority of the task */
    &flagsTaskHandle,                                                                                                    /* Task handle. */
    0                                                                                                                    /* Core where the task should run */
  );
  // INICIALIZACION DE freeRTOS END ========================================================================================================================

  Serial.println(F("Temporizador establecido para 5s, la primera medida se tomara tras dicho tiempo"));
  Serial.println(F("============= SETUP END ============"));
}
// SETUP END -----------------------------------------------------------------------------------------------------------------------------------------------

void loop() {
  delay(1000);
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION CALLBACK - FUNCION A LA QUE LLEGAN LOS MENSAJES DE LOS TOPICOS SUSCRITOS PARA HACER CAMBIOS EN EL MICRO, ES COMO UN SEGUNDO LOOP
// ---------------------------------------------------------------------------------------------------------------------------------------------------------
void callback(char* topic, byte* message, unsigned int length){                                                          // Funcion que recibe el topico MQTT, el mensaje y la longitud del mismo
  String messageSub;

  if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
    Serial.print(F("\t\tMessage arrived on topic: "));
    Serial.print(topic);                                                                                                 // En 'topic' se guarda el nombre del topic. Por ejemplo 'moya/luces'
    Serial.print(F(". Message: "));
    for (int i = 0; i < length; i++) {                                                                                   // Bucle para printear el mensaje (caracter a caracter)
      Serial.print((char)message[i]);
      messageSub += (char)message[i];                                                                                    // En 'messageTemp' se carga el contenido de 'message', el cual se reinicia tras cada iteracion del 'loop'
    }
    Serial.println();
    xSemaphoreGive(semaphoreSerial);
  }

  // =======================================================================================================================================================
  // Feel free to add more if statements to control more GPIOs with MQTT
  // =======================================================================================================================================================
  if(String(topic) == mqttTopicSub){                                                                                     // Again, ensure the topic is the desired one
    // Parse the JSON object -------------------------------------------------------------------------------------------------------------------------------
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, messageSub);
    if(error){
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.print("\t\tJSON deserialization failed: ");
        Serial.println(error.c_str());
        xSemaphoreGive(semaphoreSerial);
      }
      return;
    }

    // Extract LED -----------------------------------------------------------------------------------------------------------------------------------------
    if(doc.containsKey("led")){
      if(doc["led"].as<bool>()){
        ledStateON = true;
      }else{
        ledStateOFF = true;
      }
    }

     // Extract OLED ---------------------------------------------------------------------------------------------------------------------------------------
    if(doc.containsKey("oled")){
      if(doc["oled"].as<bool>()){
        oledStateON = true;
      }else{
        oledStateOFF = true;
      }
    }

    // Extract buzzer --------------------------------------------------------------------------------------------------------------------------------------
    if(doc.containsKey("buzzer")){
      if(doc["buzzer"].as<bool>()){
        claxonState = true;
      }
    }

    // Extract joystick values -----------------------------------------------------------------------------------------------------------------------------
    joystickData joyData;
    if(doc.containsKey("xAxis")){
      joyData.potXaxis = doc["xAxis"];
    }
    if (doc.containsKey("yAxis")) {
      joyData.potYaxis = doc["yAxis"];
    }

    if(!xQueueSend(joystickQueue, &joyData, portMAX_DELAY)){                                                             // Send data to queue
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("Cola del joystick ha fallado!"));
        xSemaphoreGive(semaphoreSerial);
      }
    }
  }
  // -------------------------------------------------------------------------------------------------------------------------------
  // FUNCION CALLBACK END ==================================================================================================================================
}
// ---------------------------------------------------------------------------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION RECONNECT - FUNCION QUE ESTABLECE LA CONNEXION POR MQTT PARA RECIBIR LOS MENSAJES DE LOS TOPICOS A LOS QUE SE ESTA SUSCRITO
// ---------------------------------------------------------------------------------------------------------------------------------------------------------
void reconnect() {
  while(!mqttClient.connected()){                                                                                        // Itera hasta reconectar, O SIMPLEMENTE CONECTAR
    if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
      Serial.print(F("Attempting MQTT connection..."));
      xSemaphoreGive(semaphoreSerial);
    }
    if(mqttClient.connect("ESP32_Moya", access_token, NULL)){                                                            // Conexion establecida con el dispositivo MQTT. CUIDADO QUE AQUI SE PUEDE AÑADIR ,"USER", "PASSWORD"
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("connected"));
        xSemaphoreGive(semaphoreSerial);
      }

      // ===================================================================================================================================================
      // Feel free to add more if statements to control more GPIOs with MQTT
      // ===================================================================================================================================================
      mqttClient.subscribe(mqttTopicSub);
      // ===================================================================================================================================================

    }else{
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.print(F("failed, rc="));                                                                                  // Si no se establece conexion
        Serial.print(mqttClient.state());
        Serial.println(F(" try again in 5 seconds"));                                                                    // Espera 5 segundos hasta reintentar la conexion
        xSemaphoreGive(semaphoreSerial);
      }
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}
// ---------------------------------------------------------------------------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION PARA CALCULAR LA ORIENTACION ACTUAL
// ---------------------------------------------------------------------------------------------------------------------------------------------------------
float current_heading(){
  sensors_event_t event;
  mag.getEvent(&event);

  float head = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.007;
  head += declinationAngle;

  if(head < 0) head += 2 * PI;                                                                                          // Correct for when signs are reversed
  if(head > 2 * PI) head -= 2 * PI;                                                                                     // Check for wrap due to addition of declination

  return head * 180 / M_PI;
}
// ORIENTACION END -----------------------------------------------------------------------------------------------------------------------------------------
