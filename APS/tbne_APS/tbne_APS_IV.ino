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
#include <DFRobot_QMC5883.h>                                                                                             // Libreria para la brujula QMC5883L

#include <Adafruit_GFX.h>                                                                                                // Libreria complementaria para el panel OLED
#include <Adafruit_SSD1306.h>                                                                                            // Libreria especifica para el panel OLED de 128x64 pixeles

#include <ESP32Servo.h>                                                                                                  // Libreria para manejos de servos para placas que usen ESP32

#include <TinyGPS++.h>                                                                                                   // Libreria para usar el GPS NEO-6M por puerto serie

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

#define FORWARD_RIGHT  120
#define FORWARD_LEFT    60

#define CLOCKWISE_RIGHT 60
#define CLOCKWISE_LEFT  60

#define COUNTERCLOCKWISE_RIGHT 120
#define COUNTERCLOCKWISE_LEFT  120

#define STOP_RIGHT      90
#define STOP_LEFT       90

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

// Macros auto-pilot -----------------------------------------------------------------------------------------------------------------------------------------
#define METERS_PER_DEGREE_LAT 111000                                              // Approximation of distance (meters per degree): 111,000 meters/degree latitude

// Macros para temporalizaciones -----------------------------------------------------------------------------------------------------------------------------
// MACROS END ================================================================================================================================================

// ===========================================================================================================================================================
// CONSTRUCTORES DE OBJETOS DE CLASE DE LIBRERIA, VARIABLES GLOBALES, CONSTANTES...
// ===========================================================================================================================================================
WiFiClientSecure secureClient;                                                                                           // Objeto de la libreria WiFiManager
PubSubClient mqttClient(secureClient);                                                                                   // Objeto de la libreria MQTT

TinyGPSPlus gps;                                                                                                         // Objeto del GPS

DFRobot_QMC5883 compass(&Wire, QMC5883_ADDRESS);

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
static struct sensorData{
  float temp, hum, alt, pres, vbat, lat, lon, veloc, pm2_5, pm10, heading;                                               // Definicion de las variables 'float'
  uint8_t sat;                                                                                                           // Variable en la que se guarda el numero de satelites en cobertura del GPS
};

// Struct para los datos del auto-pilot
static struct APData{
  float lat, lon, heading;
};

// Struct para los valores de los ejes del joystick virtual --------------------------------------------------------------------------------------------------
static struct joystickData{
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
// GLOBAL VARIABLES END ======================================================================================================================================

// ===========================================================================================================================================================
// FREERTOS CONSTRUCTORS
// ===========================================================================================================================================================
// Task handles ----------------------------------------------------------------------------------------------------------------------------------------------
static TaskHandle_t mqttTaskHandle, sensorTaskHandle, actuatorTaskHandle, flagsTaskHandle, autoPilotTaskHandle;

// Sensor queue ----------------------------------------------------------------------------------------------------------------------------------------------
static QueueHandle_t sensorQueuePub = NULL, sensorQueueAP = NULL, joystickQueue = NULL;

// Semaphore -------------------------------------------------------------------------------------------------------------------------------------------------
static SemaphoreHandle_t semaphoreSerial;
// FREERTOS CONSTRUCTORS END =================================================================================================================================

// ===========================================================================================================================================================
// FUNCTION PROTOTYPES
// ===========================================================================================================================================================
// Tasks -----------------------------------------------------------------------------------------------------------------------------------------------------
static void mqttTask(void*);
static void sensorTask(void*);
static void flagsTask(void*);
static void actuatorTask(void*);
static void autoPilotTask(void*);

// Auxiliary functions ---------------------------------------------------------------------------------------------------------------------------------------
static void callback(char*, byte*, unsigned int);
static void reconnect();
static double calculateBearing(double, double, double, double);
static float distanceTo(double, double, double, double);
// ===========================================================================================================================================================

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// MQTT TASK - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO PARA EL MQTT PARA EL PUBLISHING
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
static void mqttTask(void* pvParameters) {
  TickType_t lastReceivedTime = xTaskGetTickCount();                                                                     // Timestamp for the last data received
  const TickType_t errorThreshold = pdMS_TO_TICKS(6000);                                                                 // 6-second threshold

  while(true){
    if(!mqttClient.connected()){                                                                                         // Si no hay conexion
      reconnect();                                                                                                       // Entra la funcion de reconexion
    }
    mqttClient.loop();

    sensorData sData;
    if(!xQueueReceive(sensorQueuePub, &sData, pdMS_TO_TICKS(100))){
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
        sprintf(dataStr, "{\"temperature\":%5.2f,\"pressure\":%6.2f,\"altitude\":%4.2f,\"humidity\":%4.2f,\"battery\":%4.3f,\"latitude\":%10.6f,\"longitude\":%10.6f,\"sats\":%1d,\"speed\":%5.2f,\"PM2_5\":%6.2f,\"PM10\":%6.2f,\"heading\":%5.2f}",
                sData.temp, sData.pres, sData.alt, sData.hum, sData.vbat, sData.lat, sData.lon, sData.sat, sData.veloc, sData.pm2_5, sData.pm10, sData.heading);  // La funcion 'sprintf' de C++ se usa para poder introducir las medidas y elegir el ancho de numero y su precision. Las medidas se separan con una coma ','
        
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
static void sensorTask(void* pvParameters){
  TickType_t lastSendTime = xTaskGetTickCount();                                                                     // Timestamp for the last data received
  const TickType_t sensorQueuePubInterval = pdMS_TO_TICKS(5000);                                                         // 5-second interval
  
  uint32_t notificationValue;
  bool AUTO_PILOT = false;

  int pm25int, pm10int;
  float pm25last = 0.0, pm10last = 0.0;
  float latTemp = 0.0, lonTemp = 0.0, satTemp = 0, velocTemp = 0.0, headingTemp = 0.0;

  while(true){
    sensorData sData;
    APData aData;
    
    // Check if pilot mode has been changed -----------------------------------------------------------------------------------------------------------------
    if(xTaskNotifyWait(0, 0xFFFFFFFF, &notificationValue, pdMS_TO_TICKS(10))){
      if(notificationValue & (1 << 10)) {
        AUTO_PILOT = true;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\tAUTO-PILOT ENGAGED"));
          xSemaphoreGive(semaphoreSerial);
        }
      }
      if(notificationValue & (1 << 11)){
        AUTO_PILOT = false;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\tAUTO-PILOT DISABLED"));
          xSemaphoreGive(semaphoreSerial);
        }
      }
    }

    // Lectura del angulo de direccion respecto de N ---------------------------------------------------------------------------------------------------------
    float declinationAngle = ((26.0 / 60.0)) / (180 / PI);
    compass.setDeclinationAngle(declinationAngle);
    sVector_t mag = compass.readRaw();
    compass.getHeadingDegrees();
    headingTemp = mag.HeadingDegress;                                                                                    // Variable que guarda la orientacion del robot con respecto a N

    // Preparacion del GPS para tener cobertura satelite y poder enviar coordenadas --------------------------------------------------------------------------
    while(SerialGPS.available() > 0){                                                                                    // Mientras haya bytes disponibles en el puerto serie del GPS
      if(gps.encode(SerialGPS.read())){                                                                                  // Se confirma que haya lecturas del GPS
        if (gps.location.isValid()) {                                                                                    // Si las coordenadas recogidas son validas
          latTemp = gps.location.lat();                                                                                 // Se guardan en las variables 'lat' y 'lon' dichas coordenadas
          lonTemp = gps.location.lng();
          satTemp = gps.satellites.value();
          velocTemp = gps.speed.kmph();
        }else{                                                                                                           // En caso de que sean invalidas, se muestra un mensaje en el monitor serie
          latTemp = 0.0;                                                                                                // Por marcar un numero definido, la ubicacion se fija a 0 en ambos 'lat' y 'lon'
          lonTemp = 0.0;
          satTemp = 0;
          velocTemp = 0.0;
        }
      }
    }

    if((xTaskGetTickCount() > pdMS_TO_TICKS(15000)) && (gps.charsProcessed() < 10)){                                     // Si durante 15 segundos se cumple que los caracteres procesados del GPS son menos de 10, implica que hay un error leyendo el GPS, se reporta el fallo y se bloquea el programa hasta que se solvente el error
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("\tGPS no detectado: comprueba el cableado"));
        xSemaphoreGive(semaphoreSerial);
      }
      vTaskDelay(pdMS_TO_TICKS(15000));
    }
    
    if(AUTO_PILOT){
      aData.heading = headingTemp;
      aData.lat = latTemp;
      aData.lon = lonTemp;
      if(!xQueueSend(sensorQueueAP, &aData, 0)){
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\tCola de sensores auto-pilot ha fallado!"));
          xSemaphoreGive(semaphoreSerial);
        }
      }
    }

    if((xTaskGetTickCount() - lastSendTime) > sensorQueuePubInterval){
      sData.heading = headingTemp;
      sData.lat = latTemp;
      sData.lon = lonTemp;
      sData.sat = satTemp;
      sData.veloc = velocTemp;

      if(satTemp == 0){
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\tINVALID GPS "));
          xSemaphoreGive(semaphoreSerial);
        }
      }

      // Lectura del BME280 ------------------------------------------------------------------------------------------------------------------------------------
      sData.temp = bme.readTemperature();                                                                                   // Variable que guarda la temperatura del sensor BME280
      sData.pres = bme.readPressure() / 100.0F;                                                                             // Variable que guarda la presion del sensor BME280
      sData.alt = bme.readAltitude(SEALEVELPRESSURE_HPA);                                                                   // Variable que guarda la altitud del sensor BME280
      sData.hum = bme.readHumidity();                                                                                       // Variable que guarda la humedad del sensor BME280
      
      // Lectura de la celda 18650 del shield por medio del divisor resistivo ----------------------------------------------------------------------------------
      sData.vbat = (float)(analogRead(VBAT_PIN)) / 4095 * 2 * 3.3 * 1.1;                                                    // Variable que guarda el valor del voltaje en el conector de batería

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
          sData.pm2_5 = pm25int / 10.0;
          sData.pm10 = pm10int / 10.0;

          pm25last = sData.pm2_5;
          pm10last = sData.pm10;
        }else{
          if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
            Serial.println(F("\tInvalid ending byte from SDS011"));
            xSemaphoreGive(semaphoreSerial);
          }
          sData.pm2_5 = pm25last;
          sData.pm10 = pm10last;
        }
      }else{
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\tNot enough data from SDS011"));
          xSemaphoreGive(semaphoreSerial);
        }
      }

      if(!xQueueSend(sensorQueuePub, &sData, 0)) {
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\tCola de sensores publisher ha fallado!"));
          xSemaphoreGive(semaphoreSerial);
        }
      }
      lastSendTime = xTaskGetTickCount();
    }
    // -------------------------------------------------------------------------------------------------------------------------------------------------------

    vTaskDelay(pdMS_TO_TICKS(500));                                                                                     // Read every 250 seconds, the most demanding queue
  }
}
// SENSOR TASK END -------------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FLAGS TASK - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO QUE GESTIONA EL I/O DIGITAL CON VOLATILE BOOL FLAGS
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
static void flagsTask(void* pvParameters){  
  bool FIRE_ALARM = false, HOT_ALARM = false, COLD_ALARM = false, POLLUTION_ALARM = false, RAIN_ALARM = false;
  
  uint32_t notificationValue;

  while(true){
    if(xTaskNotifyWait(0, 0xFFFFFFFF, &notificationValue, pdMS_TO_TICKS(1000))){
      oled.setTextSize(1);                                                                                               // Se selecciona el tamaño de la letra

      // FIRE ALARM ON ---------------------------------------------------------------------------------------------------------------------------------------
      if (notificationValue & (1 << 0)) {
        FIRE_ALARM = true;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tFIRE ALARM ON"));
          xSemaphoreGive(semaphoreSerial);
        }
        oled.setCursor(10, 5);                                                                                           // Se indica el pixel donde se quiere escribir (X,Y)
        oled.print("POTENTIAL FIRE");                                                                                    // Se escribe un string
      }

      // FIRE ALARM OFF --------------------------------------------------------------------------------------------------------------------------------------
      if (notificationValue & (1 << 1)) {
        FIRE_ALARM = false;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tFIRE ALARM OFF"));
          xSemaphoreGive(semaphoreSerial);
        }
        oled.fillRect(10, 5, 120, 10, BLACK);                                                                            // Clear the line (x, y, width, height, color)
      }

      // HOT ALARM ON ----------------------------------------------------------------------------------------------------------------------------------------
      if (notificationValue & (1 << 2)) {
        HOT_ALARM = true;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tHOT ALARM ON"));
          xSemaphoreGive(semaphoreSerial);
        }
        oled.setCursor(10, 17);
        oled.print("EXTREME HOT");
      }

      // HOT ALARM OFF ---------------------------------------------------------------------------------------------------------------------------------------
      if (notificationValue & (1 << 3)) {
        HOT_ALARM = false;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tHOT ALARM OFF"));
          xSemaphoreGive(semaphoreSerial);
        }
        oled.fillRect(10, 17, 120, 10, BLACK);
      }

      // COLD ALARM ON ---------------------------------------------------------------------------------------------------------------------------------------
      if (notificationValue & (1 << 4)) {
        COLD_ALARM = true;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tCOLD ALARM ON"));
          xSemaphoreGive(semaphoreSerial);
        }
        oled.setCursor(10, 29);
        oled.print("EXTREME COLD");
      }

      // COLD ALARM OFF --------------------------------------------------------------------------------------------------------------------------------------
      if (notificationValue & (1 << 5)) {
        COLD_ALARM = false;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tCOLD ALARM OFF"));
          xSemaphoreGive(semaphoreSerial);
        }
        oled.fillRect(10, 29, 120, 10, BLACK);
      }
      
      // POLLUTION ALARM ON ----------------------------------------------------------------------------------------------------------------------------------
      if (notificationValue & (1 << 6)) {
        POLLUTION_ALARM = true;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tPOLLUTION ALARM ON"));
          xSemaphoreGive(semaphoreSerial);
        }
        oled.setCursor(10, 41);
        oled.print("HIGH POLLUTION");
      }

      // POLLUTION ALARM OFF ---------------------------------------------------------------------------------------------------------------------------------
      if (notificationValue & (1 << 7)) {
        POLLUTION_ALARM = false;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tPOLLUTION ALARM OFF"));
          xSemaphoreGive(semaphoreSerial);
        }
        oled.fillRect(10, 41, 120, 10, BLACK);
      }

      // RAIN ALARM ON ---------------------------------------------------------------------------------------------------------------------------------------
      if (notificationValue & (1 << 8)) {
        RAIN_ALARM = true;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tRAIN ALARM ON"));
          xSemaphoreGive(semaphoreSerial);
        }
        oled.setCursor(10, 53);
        oled.print("HEAVY RAIN");
      }

      // RAIN ALARM OFF ---------------------------------------------------------------------------------------------------------------------------------------
      if (notificationValue & (1 << 9)) {
        RAIN_ALARM = false;
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tRAIN ALARM OFF"));
          xSemaphoreGive(semaphoreSerial);
        }
        oled.fillRect(10, 53, 120, 10, BLACK);
      }

      oled.display();                                                                                                    // Mostrar el buffer actual
    }

    // COMMON TO ALL ALARMS ---------------------------------------------------------------------------------------------------------------------------------
    if(FIRE_ALARM || HOT_ALARM || COLD_ALARM || POLLUTION_ALARM || RAIN_ALARM){
      for(int i = 0; i <= 255; i += 1){
        ledcWrite(LED_PIN, i);
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      for(int i = 0; i <= 255; i += 1){
        ledcWrite(BUZZER_PIN, i);
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      for(int i = 255; i >= 0; i -= 1){
        ledcWrite(LED_PIN, i);
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      for(int i = 255; i >= 0; i -= 1){
        ledcWrite(BUZZER_PIN, i);
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }else{
      oled.clearDisplay();                                                                                             // Se limpia el buffer del OLED
      oled.display();                                                                                                  // Se printea el buffer que, como no es nada, se apaga
    }

    vTaskDelay(pdMS_TO_TICKS(100));                                                                                      // Small delay
  }
}
// FLAGS TASK END --------------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// ACTUATOR TASK - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO PARA EJECUTAR LAS ORDENES DE LOS SERVOS
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
static void actuatorTask(void* pvParameters){
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
// AUTO-PILOT TASK - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO PARA EJECUTAR EL AUTOPILOT
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
static void autoPilotTask(void* pvParameters){
  const double checkpoints[][2] = {
    {43.527208, -5.632747},
    {43.526658, -5.631508},
    {43.525135, -5.632324},
    {43.525510, -5.630115},
    {43.527104, -5.630602},
    {43.527208, -5.632747}
  };

  const int totalCheckpoints = sizeof(checkpoints) / sizeof(checkpoints[0]);        // Autoadjastable checkpoint number the first 'sizeof' is the total amount of elements while the second is always 16 bytes, as it is the amount of elements to define a coordinate
  int currentCheckpoint = 0;                                                        // Checkpoint controller

  while (true) {
    APData aData;

    if(!xQueueReceive(sensorQueueAP, &aData, portMAX_DELAY)){
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("\t\t\tError recibiendo datos de la cola del auto-pilot"));
        xSemaphoreGive(semaphoreSerial);
      }
    }else{
      // Get current and target coordinates
      double currentLat = aData.lat;
      double currentLon = aData.lon;
      double targetLat = checkpoints[currentCheckpoint][0];
      double targetLon = checkpoints[currentCheckpoint][1];

      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.print(F("\t\t\tCurrent Location: "));
        Serial.print(currentLat, 6);
        Serial.print(F(", "));
        Serial.println(currentLon, 6);
        xSemaphoreGive(semaphoreSerial);
      }

      // Calculate bearing to target
      double targetBearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);

      // Get current heading from queue
      double currentHeading = aData.heading;

      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.print(F("\t\t\tTarget Bearing: "));
        Serial.println(targetBearing);                                                // Next checkpoint orientation angle
        Serial.print(F("Current Heading: "));
        Serial.println(currentHeading);                                               // Current orientation angle
        xSemaphoreGive(semaphoreSerial);
      }
      
      // Adjust direction ----------------------------------------------------------
      double bearingDifference = targetBearing - currentHeading;                    // Bearing difference is the substraction of the target orientation and the current one
      if(bearingDifference > 180) bearingDifference -= 360;                         // Bearings must be in the [-180, 180] degree range, so if it exceeds 180º, it must be replaced by its equivalent negative counterpart in the valid range
      if(bearingDifference < -180) bearingDifference += 360;                        // Same for those bearings exceeding -180

      if(abs(bearingDifference) > 10){                                              // If the bearing difference is significant (10 degrees or more in any orientation)
        if(bearingDifference > 0){                                                  // If the bearing difference is positive, turn to the opposite direction, RIGHT
          miServoRight.write(CLOCKWISE_RIGHT);
          miServoLeft.write(CLOCKWISE_LEFT);
          if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
            Serial.println(F("\t\t\tCLOCKWISE COUNTERSTEER"));
            xSemaphoreGive(semaphoreSerial);
          }
        }else{                                                                      // Do the same if the difference is negative
          miServoRight.write(COUNTERCLOCKWISE_RIGHT);
          miServoLeft.write(COUNTERCLOCKWISE_LEFT);
          if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
            Serial.println(F("\t\t\tCOUNTERCLOCKWISE COUNTERSTEER"));
            xSemaphoreGive(semaphoreSerial);
          }
        }
      }else{                                                                        // If not, no correction is needed, so move towards the next checkpoint
        miServoRight.write(FORWARD_RIGHT);
        miServoLeft.write(FORWARD_LEFT);
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tMOVING FORWARD"));
          xSemaphoreGive(semaphoreSerial);
        }
      }

      // Check if close to the target checkpoint -----------------------------------
      if(distanceTo(currentLat, currentLon, targetLat, targetLon) < 5.0){           // If the robot is 5 meters or less close to the checkpoint, it is considered to be there
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("Checkpoint reached."));
          xSemaphoreGive(semaphoreSerial);
        }
        miServoRight.write(STOP_RIGHT);
        miServoLeft.write(STOP_LEFT);
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("\t\t\tSTOP MOTORS"));
          xSemaphoreGive(semaphoreSerial);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));                                            // The robot stops for a second

        currentCheckpoint++;                                                        // The next checkpoint is loaded
        if(currentCheckpoint >= totalCheckpoints){                                  // If it is the last checkpoint
          if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
            Serial.println(F("\t\t\tAll checkpoints completed. Stopping robot."));             // Stop forever
            xSemaphoreGive(semaphoreSerial);
          }

          while(1){
            miServoRight.write(STOP_RIGHT);
            miServoLeft.write(STOP_LEFT);
            if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
              Serial.println(F("\t\t\tSTOP MOTORS"));
              xSemaphoreGive(semaphoreSerial);
            }
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(250));                                                                                      // Small delay
  }
}
// AUTO-PILOT TASK END -----------------------------------------------------------------------------------------------------------------------------------------

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

  // QMC5883L ----------------------------------------------------------------------------------------------------------------------------------------------
  if(!compass.begin()){
    Serial.println(F("Could not find a valid 5883 sensor, check wiring!"));
    while(1);
  }

  if(compass.isQMC()){
    Serial.println(F("Initialize QMC5883"));
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
  sensorQueueAP = xQueueCreate(1, sizeof(APData));
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

  xTaskCreatePinnedToCore(
    autoPilotTask,                                                                                                       /* Function to implement the task */
    "autoPilotTask",                                                                                                         /* Name of the task */
    5000,                                                                                                                /* Stack size in bytes */
    NULL,                                                                                                                /* Task input parameter */
    1,                                                                                                                   /* Priority of the task */
    &autoPilotTaskHandle,                                                                                                /* Task handle. */
    0                                                                                                                    /* Core where the task should run */
  );

  vTaskSuspend(autoPilotTaskHandle);                                                                                     // Autopilot starts as disabled
  // INICIALIZACION DE freeRTOS END ========================================================================================================================
  
  Serial.println(F("Temporizador establecido para 5s, la primera medida se tomara tras dicho tiempo"));
  Serial.println(F("============= SETUP END ============"));
  
  oled.setTextSize(3);
  oled.setCursor(40, 17);
  oled.print("DEPLOYED");
  oled.display();
  delay(1000);
  oled.clearDisplay();
}
// SETUP END -----------------------------------------------------------------------------------------------------------------------------------------------

void loop() {
  delay(1000);
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION CALLBACK - FUNCION A LA QUE LLEGAN LOS MENSAJES DE LOS TOPICOS SUSCRITOS PARA HACER CAMBIOS EN EL MICRO, ES COMO UN SEGUNDO LOOP
// ---------------------------------------------------------------------------------------------------------------------------------------------------------
static void callback(char* topic, byte* message, unsigned int length){                                                   // Funcion que recibe el topico MQTT, el mensaje y la longitud del mismo
  static bool AUTO_PILOT = false;                                                                                        // Must be declared as 'static' as, even if callback belongs to mqttTask, it would be out of scope, so being static allows to keep the last state of the mode when the function is executed
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
        Serial.print(F("\t\tJSON deserialization failed: "));
        Serial.println(error.c_str());
        xSemaphoreGive(semaphoreSerial);
      }
      return;
    }

    // Extract fire alarm ----------------------------------------------------------------------------------------------------------------------------------
    if(doc.containsKey("alarm_fire")){
      if(doc["alarm_fire"].as<bool>()){
        xTaskNotify(flagsTaskHandle, 1 << 0, eSetBits);
      }else{
        xTaskNotify(flagsTaskHandle, 1 << 1, eSetBits);
      }
    }

     // Extract hot alarm ----------------------------------------------------------------------------------------------------------------------------------
    if(doc.containsKey("alarm_hot")){
      if(doc["alarm_hot"].as<bool>()){
        xTaskNotify(flagsTaskHandle, 1 << 2, eSetBits);
      }else{
        xTaskNotify(flagsTaskHandle, 1 << 3, eSetBits);
      }
    }

     // Extract cold alarm ---------------------------------------------------------------------------------------------------------------------------------
    if(doc.containsKey("alarm_cold")){
      if(doc["alarm_cold"].as<bool>()){
        xTaskNotify(flagsTaskHandle, 1 << 4, eSetBits);
      }else{
        xTaskNotify(flagsTaskHandle, 1 << 5, eSetBits);
      }
    }

     // Extract pollution alarm ----------------------------------------------------------------------------------------------------------------------------
    if(doc.containsKey("alarm_pollution")){
      if(doc["alarm_pollution"].as<bool>()){
        xTaskNotify(flagsTaskHandle, 1 << 6, eSetBits);
      }else{
        xTaskNotify(flagsTaskHandle, 1 << 7, eSetBits);
      }
    }

     // Extract pollution alarm ----------------------------------------------------------------------------------------------------------------------------
    if(doc.containsKey("alarm_rain")){
      if(doc["alarm_rain"].as<bool>()){
        xTaskNotify(flagsTaskHandle, 1 << 8, eSetBits);
      }else{
        xTaskNotify(flagsTaskHandle, 1 << 9, eSetBits);
      }
    }

    // Extract auto-pilot engager ----------------------------------------------------------------------------------------------------------------------------
    if(doc.containsKey("auto_pilot")){
      if(doc["auto_pilot"].as<bool>()){
        xTaskNotify(sensorTaskHandle, 1 << 10, eSetBits);

        vTaskResume(autoPilotTaskHandle);
        vTaskSuspend(actuatorTaskHandle);

        AUTO_PILOT = true;
      }else{
        xTaskNotify(sensorTaskHandle, 1 << 11, eSetBits);

        vTaskResume(actuatorTaskHandle);
        vTaskSuspend(autoPilotTaskHandle);

        AUTO_PILOT = false;
      }
    }

    // Extract joystick values -----------------------------------------------------------------------------------------------------------------------------
    if(!AUTO_PILOT && (doc.containsKey("xAxis") || doc.containsKey("yAxis"))){                                             // Joystick/manual mode is only accessible if auto-pilot is not enabled
      joystickData joyData;
      if(doc.containsKey("xAxis")){
        joyData.potXaxis = doc["xAxis"];
      }
      if(doc.containsKey("yAxis")){
        joyData.potYaxis = doc["yAxis"];
      }

      if(!xQueueSend(joystickQueue, &joyData, portMAX_DELAY)){                                                             // Send data to queue
        if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
          Serial.println(F("Cola del joystick ha fallado!"));
          xSemaphoreGive(semaphoreSerial);
        }
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
static void reconnect(){
  while(!mqttClient.connected()){                                                                                        // Itera hasta reconectar, O SIMPLEMENTE CONECTAR
    if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
      Serial.print(F("Attempting MQTT connection..."));
      xSemaphoreGive(semaphoreSerial);
    }
    if(mqttClient.connect("ESP32_Moya", access_token, NULL)){                                                            // Conexion establecida con el dispositivo MQTT. CUIDADO QUE AQUI SE PUEDE AÑADIR ,"USER", "PASSWORD"
      if(xSemaphoreTake(semaphoreSerial, portMAX_DELAY)){
        Serial.println(F("Connected to MQTT!"));
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
// BEARING COMPUTATION
// ---------------------------------------------------------------------------------------------------------------------------------------------------------
static double calculateBearing(double lat1, double lon1, double lat2, double lon2){     // Calculate straight-line bearing
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  double dLon = lon2 - lon1;                                                      // As bearing is an angle, this function works with latitude and longitude directly as they are angles too
  double dLat = lat2 - lat1;
  
  double bearing = atan2(dLon, dLat) * 180 / PI;                                  // Convert radians to degrees
  if(bearing < 0) bearing += 360;                                                 // Normalize to 0-360 degrees
  return bearing;
}
// BEARING COMPUTATION END ---------------------------------------------------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------------------------------------------------------------------------
// DISTANCE TO THE NEXT CHECKPOINT COMPUTATION
// ---------------------------------------------------------------------------------------------------------------------------------------------------------
static float distanceTo(double lat1, double lon1, double lat2, double lon2){             // Calculate Euclidean distance
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  const float METERS_PER_DEGREE_LON = METERS_PER_DEGREE_LAT * cos(lat1 * PI / 180);  // Adjust longitude distance based on latitude accounting for the Earth's curvature

  float dx = dLat * METERS_PER_DEGREE_LAT;                                        // Difference in latitude converted to meters
  float dy = dLon * METERS_PER_DEGREE_LON;                                        // Difference in longitude converted to meters

  return sqrt(dx * dx + dy * dy);                                                 // Pythagorean theorem
}
// DISTANCE TO THE NEXT CHECKPOINT COMPUTATION END ---------------------------------------------------------------------------------------------------------
