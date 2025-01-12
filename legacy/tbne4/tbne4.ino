/* ***********************************************************************************************************************************************************
EJEMPLO - PLANTILLA DE DISPOSITIVO PUBLISH/SUBSCRIBE MQTT: este skecth sirve para programar un robot de dos ruedas que cuenta con un sensor climatico
BME280, un GPS NEO-6M y un sensor de PM SDS011 que publica las medidas cada 5 segundos en el servidor MQTT https://emqx.broker.io para ser procesadas en
NodeRED. Además, en NodeRED se crean los topicos necesarios para controlar la pantalla OLED, el LED, los servos de rotacion continua y el zumbador a bordo
del robot, que reciben los parametros de un dashboard de NodeRED por medio de estar suscrito a los topicos donde se envian dichos parametros.
*********************************************************************************************************************************************************** */

// ===========================================================================================================================================================
// INLCUSION DE LIBRERIAS
// ===========================================================================================================================================================
#include <Arduino.h>                                                                                                                        // Libreria de Arduino para garantizar que se aplican funciones propias tambien en la ESP32 S3

#include <WiFiManager.h>                                                                                                                    // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>                                                                                                                   // Libreria para MQTT

#include <Wire.h>                                                                                                                           // Libreria del bus I2C

#include <Adafruit_Sensor.h>                                                                                                                // Libreria para usar sensores de Adafruit
#include <Adafruit_BME280.h>                                                                                                                // Libreria del BME280

#include <Adafruit_GFX.h>                                                                                                                   // Libreria complementaria para el panel OLED
#include <Adafruit_SSD1306.h>                                                                                                               // Libreria especifica para el panel OLED de 128x64 pixeles

#include <ESP32Servo.h>                                                                                                                     // Libreria para manejos de servos para placas que usen ESP32

#include <TinyGPS++.h>                                                                                                                      // Libreria para usar el GPS NEO-6M por puerto serie

#include <Adafruit_HMC5883_U.h>                                                                                                             // Libreria para usar la brujula HMC5883L

#include <SoftwareSerial.h>                                                                                                                 // Libreria para crear puertos serie virtuales en GPIO comunes
// ===========================================================================================================================================================

// ===========================================================================================================================================================
// MACROS (de ser necesarias)
// ===========================================================================================================================================================
// Macros OLED I2C -------------------------------------------------------------------------------------------------------------------------------------------
#define OLED_ADDR       0x3C                                                                                                                // Direccion I2C
#define SCREEN_WIDTH    128                                                                                                                 // Pixeles de ancho
#define SCREEN_HEIGHT   64                                                                                                                  // Pixeles de alto
#define OLED_RESET      -1                                                                                                                  // Reset del OLED
#define SDA_OLED        18                                                                                                                  // Se asignan los pines I2C (SDA, SCL) del panel OLED. ESTA CONEXION SDA, SCL NO ES ACCESIBLE DESDE EL PINOUT, ES UNA CONEXION INTERNA
#define SCL_OLED        17

// Macros GPS ------------------------------------------------------------------------------------------------------------------------------------------------
#define SerialGPS       Serial1                                                                                                             // Se crea un nuevo puerto serie para el GPS (el predeterminado, 'Serial', se usa para el monitor serie)
#define GPS_RX_PIN      44                                                                                                                  // Pin RX del ESP32 al pin TX del GPS
#define GPS_TX_PIN      43                                                                                                                  // Pin TX del ESP32 al pin RX del GPS
#define GPS_BAUD_RATE   9600                                                                                                                // Baudios para la comunicacion serie del GPS

// Macros SDS011 ---------------------------------------------------------------------------------------------------------------------------------------------
#define SDS_RX          47                                                                                                                  // Pin RX virtual del ESP32 al pin TX del SDS011
#define SDS_TX          48                                                                                                                  // Pin TX virtual del ESP32 al pin RX del SDS011

// Macros servos ---------------------------------------------------------------------------------------------------------------------------------------------
#define PIN_SERVO_RIGHT 45                                                                                                                  // Se asignan los pines a los que se conectan los cables 'DATA' de cada servo
#define PIN_SERVO_LEFT  41

// Macros BME280 I2C -----------------------------------------------------------------------------------------------------------------------------------------
#define SDA_BME         38                                                                                                                  // Se asignan los pines I2C (SDA, SCL) del BME280. ESTOS SI SON ACCESIBLES DESDE EL PINOUT Y SE PODRIA USAR REALMENTE COMO UN BUS I2C AL QUE CONECTARLE VARIOS DISPOSITIVOS
#define SCL_BME         39
#define SEALEVELPRESSURE_HPA (1015)                                                                                                         // Referencia de la presion atmosferica al nivel del mar

// Macros luz LED --------------------------------------------------------------------------------------------------------------------------------------------
#define LED_PIN          40
#define LED_CHANNEL      0                                                                                                                  // Canal del LED que se controla por PWM
#define FREQUENCY        5000                                                                                                               // Frecuencia de la señal PWM
#define RESOLUTION       8                                                                                                                  // Resolucion del PWM

// Macros medidor del voltaje de la bateria ------------------------------------------------------------------------------------------------------------------
#define VBAT_PIN        1                                                                                                                   // Pin internamente conectado a un circuito que lee el voltaje de la bateria en el conector inferior de la LilyGO

// Macros buzzer ---------------------------------------------------------------------------------------------------------------------------------------------
#define BUZZER_PIN      42
#define DURATION        500                                                                                                                 // Duracion de las notas del buzzer en ms

// Macros para temporalizaciones -----------------------------------------------------------------------------------------------------------------------------
#define TIMER_DELAY     5000                                                                                                                // Se inicializa el intervalo de publicacion de datos
// ===========================================================================================================================================================

// ===========================================================================================================================================================
// CONSTRUCTORES DE OBJETOS DE CLASE DE LIBRERIA, VARIABLES GLOBALES, CONSTANTES...
// ===========================================================================================================================================================
const char* mqtt_server = "broker.hivemq.com";                                                                                                 // Broker MQTT. Se ha elegido EMQX por ser gratuito y robusto
WiFiClient espClient;                                                                                                                       // Objeto de la libreria WiFiManager
PubSubClient client(espClient);                                                                                                             // Objeto de la libreria MQTT

//HardwareSerial SerialGPS(1);                                                                                                                // Se inicializa un segundo puerto serie para el GPS
TinyGPSPlus gps;                                                                                                                            // Objeto del GPS

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

SoftwareSerial sds(SDS_RX, SDS_TX);                                                                                                         // Inicializacion del puerto serie virtual para el sensor SDS011

Adafruit_BME280 bme;                                                                                                                        // Objeto del sensor BME280

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);                                                                     // Objeto del OLED

Servo miServoRight;                                                                                                                         // Objeto del servo derecho
Servo miServoLeft;                                                                                                                          // Objeto del servo izquierdo

struct sensorData{
  float temp, hum, alt, pres, vbat, lat, lon, veloc, pm2_5, pm10, heading;                                                                  // Definicion de las variables 'float'
  uint8_t sat;                                                                                                                              // Variable en la que se guarda el numero de satelites en cobertura del GPS
};

struct joystickData{
  int potXaxis, potYaxis;
};

int pm25int, pm10int;                                                                                                   // Definicion de las variables 'int'

unsigned long lastTime = 0;                                                                                                                 // Se inicializa la variable en la que se guarda el tiempo en milisegundos tras cada iteracion del loop

volatile bool oledStateON = false, oledStateOFF = false, ledStateON = false, ledStateOFF = false, claxonState = false;
bool claxonON = false;                                                                                                                     // Flag to track OLED state
unsigned long lastOledUpdateTime = 0;                                                                                                       // Timer for OLED updates
unsigned long claxonStartTime = 0;
//unsigned long lastJoystickTime = 0;
// ===========================================================================================================================================================

// ===========================================================================================================================================================
// FREERTOS CONSTRUCTORS
// ===========================================================================================================================================================
// Task handles
TaskHandle_t mqttTaskHandle, sensorTaskHandle, actuatorTaskHandle;

// Sensor queue
QueueHandle_t sensorQueue, joystickQueue;
// ===========================================================================================================================================================

// ===========================================================================================================================================================
// FUNCTION FORWARD DECLARATIONS
// ===========================================================================================================================================================
void callback(char*, byte*, unsigned int);
void reconnect();
void updateOledDisplay();
// ===========================================================================================================================================================

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// MQTT TASK - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO PARA EL MQTT PARA EL PUBLISHING
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void mqttTask(void *parameter){
  while(true){
    sensorData data;

    if(xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdTRUE){
      if(!client.connected()){                                                                                                                  // Si no hay conexion
        reconnect();                                                                                                                            // Entra la funcion de reconexion
      }
      client.loop();


      if(WiFi.status()== WL_CONNECTED){                                                                                                       // Check WiFi connection status
        // ===================================================================================================================================================
        // PREPARACION DE LAS MEDICIONES DEL SENSOR Y CONVERSION A STRING DE CARACTERES PARA ENVIARSE POR MQTT
        // =====================================================================================================================================================      
        char dataStr[120];                                                                                                                    // Se crea un string de caracteres para guardar ambas medidas, se reservan 60 espacios

        sprintf(dataStr, "%5.2f, %6.2f, %4.2f, %4.2f, %4.3f, %10.6f, %10.6f, %1d, %5.2f, %6.2f, %6.2f, %5.2f", data.temp, data.pres, data.alt, data.hum, data.vbat, data.lat, data.lon, data.sat, data.veloc, data.pm2_5, data.pm10, data.heading);  // La funcion 'sprintf' de C++ se usa para poder introducir las medidas y elegir el ancho de numero y su precision. Las medidas se separan con una coma ','
        client.publish("moya/sensores", dataStr);                                                                                             // Se publica el string con los datos de los sensores en el topico 'moya/sensores'
        Serial.println(dataStr);                                                                                                              // Muestra en el serial de Arduino el string
        // =====================================================================================================================================================

      }else{
        Serial.println(F("WiFi Disconnected"));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));                                                                                                       // Small delay to prevent excessive CPU usage
  }
}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// SENSOR TASK - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO PARA LEER LOS SENSORES
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void sensorTask(void *parameter){
  while(true){
    sensorData data;

    data.temp = bme.readTemperature();                                                                                                         // Variable que guarda la temperatura del sensor BME280
    data.pres = bme.readPressure() / 100.0F;                                                                                                   // Variable que guarda la presion del sensor BME280
    data.alt = bme.readAltitude(SEALEVELPRESSURE_HPA);                                                                                         // Variable que guarda la altitud del sensor BME280
    data.hum = bme.readHumidity();                                                                                                             // Variable que guarda la humedad del sensor BME280
    data.vbat = (float)(analogRead(VBAT_PIN)) / 4095*2*3.3*1.1;                                                                                // Variable que guarda el valor del voltaje en el conector de batería
    data.heading = current_heading();                                                                                                                        // Placeholder for compass

    // Preparacion del GPS para tener cobertura satelite y poder enviar coordenadas ------------------------------------------------------------------------
    while(SerialGPS.available() > 0){                                                                                                     // Mientras haya bytes disponibles en el puerto serie del GPS
      if(gps.encode(SerialGPS.read())){                                                                                                   // Se confirma que haya lecturas del GPS
        if(gps.location.isValid()){                                                                                                       // Si las coordenadas recogidas son validas
          data.lat = gps.location.lat();                                                                                                  // Se guardan en las variables 'lat' y 'lon' dichas coordenadas
          data.lon = gps.location.lng();
          data.sat = gps.satellites.value();
          data.veloc = gps.speed.kmph();
        }else{                                                                                                                            // En caso de que sean invalidas, se muestra un mensaje en el monitor serie
          data.lat = 0.0;                                                                                                                      // Por marcar un numero definido, la ubicacion se fija a 0 en ambos 'lat' y 'lon'
          data.lon = 0.0;
          data.sat = 0;
          data.veloc = 0.0;
          Serial.print(F("INVALID GPS "));
        }
      }
    }

    // Preparacion del SDS011 para mandar los bytes que contienen las medidas de PM2.5 y PM10 --------------------------------------------------------------
    while(sds.available() && sds.read() != 0xAA){}                                                                                        // Look for the starting byte of the SDS011 data frame
    if(sds.available()){
      Serial.println(F("Data available from SDS011..."));
    }
    byte buffer[10];                                                                                                                      // Once we have the starting byte, attempt to read the next 9 bytes
    buffer[0] = 0xAA;                                                                                                                     // The starting byte we already found
    if(sds.available() >= 9){
      sds.readBytes(&buffer[1], 9);
      if(buffer[9] == 0xAB){                                                                                                              // Check if the last byte is the correct ending byte
        pm25int = (buffer[3] << 8) | buffer[2];
        pm10int = (buffer[5] << 8) | buffer[4];
        data.pm2_5 = pm25int / 10.0;
        data.pm10 = pm10int / 10.0;
      }else{
        Serial.println(F("Invalid ending byte from SDS011"));
        data.pm2_5 = 0.0;
        data.pm10 = 0.0;
      }
    }else{
      Serial.println(F("Not enough data from SDS011"));
    }

    if(millis() > 15000 && gps.charsProcessed() < 10){                                                                                    // Si durante 15 segundos se cumple que los caracteres procesados del GPS son menos de 10, implica que hay un error leyendo el GPS, se reporta el fallo y se bloquea el programa hasta que se solvente el error
      Serial.println(F("GPS no detectado: comprueba el cableado"));
      delay(15000);
    }
    // -----------------------------------------------------------------------------------------------------------------------------------------------------
  
    xQueueSend(sensorQueue, &data, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(5000)); // Read every 5 seconds
  }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// ACTUATOR TASK - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO PARA EJECUTAR LAS ORDENES DE LOS ACTUADORES
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void actuatorTask(void *parameter){
  while(true){
    // ON LED ------------------------------------------------------------------------------------------------------------------------------------------------
    if(ledStateON){
      for(int i = 0; i <= 255; i += 1){
        ledcWrite(LED_PIN,i);
        delay(1);
      }
      ledStateON = false;
    }

    // OFF LED -----------------------------------------------------------------------------------------------------------------------------------------------
    if(ledStateOFF){
      for(int i = 255; i >= 0; i-= 1){
        ledcWrite(LED_PIN,i);
        delay(1);
      }
      ledStateOFF = false;
    }

    // ON OLED -----------------------------------------------------------------------------------------------------------------------------------------------
    if(oledStateON && (millis() - lastOledUpdateTime >= 5000)){                                                                                 // Actualiza el OLED si esta encendido y han pasado 5 segundos
      updateOledDisplay();
      lastOledUpdateTime = millis();
    }
    
    // OFF OLED ---------------------------------------------------------------------------------------------------------------------------------------------
    if(oledStateOFF){
      oledStateON = false;
      oled.clearDisplay();                                                                                                                  // Se limpia el buffer del OLED
      oled.display();                                                                                                                       // Se printea el buffer que, como no es nada, se apaga
      oledStateOFF = false;
    }

    // CLAXON FOR 500 ms -------------------------------------------------------------------------------------------------------------------------------------
    if(claxonState && !claxonON){
      ledcWriteTone(BUZZER_PIN, 500);                                                                                                        // Formerly "tone(BUZZER_PIN, 300, DURATION);"
      claxonStartTime = millis();
      claxonON = true;
      claxonState = false;
    }
    if(claxonON && millis() - claxonStartTime >= 500){
      ledcWriteTone(BUZZER_PIN, 0);
      claxonON = false;
    }

    joystickData joyData;
    if(xQueueReceive(joystickQueue, &joyData, portMAX_DELAY) == pdTRUE){
      Serial.print(F("Eje X: ")); Serial.print(joyData.potXaxis); Serial.print(F("\tEje Y: ")); Serial.println(joyData.potYaxis);

      int servoXPosition = map(joyData.potXaxis, -100, 100, 45, -45);                                                                                   // Posición neutra del eje X del joystick es 0, que corresponde a 0º en el servo. Respectivamente, -100 es 45º y 100, -45º. Devuelve valores de -45 a 45
      int servoYPosition = map(joyData.potYaxis, -100, 100, -45, 45);                                                                                   // Lo mismo para el eje Y del joystick es al revés, -100 es -45º y 100, 45º

      miServoRight.write(90 + servoYPosition + servoXPosition);                                                                                 // 90 + 45 + 0 = 135 -> Hacia adelante
      miServoLeft.write(90 - servoYPosition + servoXPosition);                                                                                  // 90 - 45 + 0 = 45  -> Hacia atras PORQUE ESTA PUESTO "ESPEJO" RESPECTO DEL OTRO EN EL ROBOT; CON ELLO SE MUEVE HACIA ADELANTE

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
    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay
  }
}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION SETUP - SOLO SE EJECUTA UNA VEZ
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // =======================================================================================================================================================
  // INICIALIZACION DE I/O
  // =======================================================================================================================================================
  // Buzzer ------------------------------------------------------------------------------------------------------------------------------------------------
  //pinMode(BUZZER_PIN, OUTPUT);                                                                                                            // El zumbador es un output digital
  ledcAttach(BUZZER_PIN, FREQUENCY, RESOLUTION);

  // LED ---------------------------------------------------------------------------------------------------------------------------------------------------
  ledcAttach(LED_PIN, FREQUENCY, RESOLUTION);                                                                                               // Inicializacion del PWM para el LED

  // Servos ------------------------------------------------------------------------------------------------------------------------------------------------
  miServoRight.attach(PIN_SERVO_RIGHT);                                                                                                     // Union del servo al pin correspondiente
  miServoLeft.attach(PIN_SERVO_LEFT);

  // GPS ---------------------------------------------------------------------------------------------------------------------------------------------------
  SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);                                                                       // Inicializacion de la comunicacion por serial para el GPS
  Serial.print(F("Testing TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());

  // SDS011 ------------------------------------------------------------------------------------------------------------------------------------------------
  sds.begin(9600);

  // OLED --------------------------------------------------------------------------------------------------------------------------------------------------
  Wire1.begin(SDA_OLED, SCL_OLED);                                                                                                          // Initialize I2C for the OLED. SDA, SCL
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.display();                                                                                                                           // Display initialization - LOGO ADAFRUIT
  delay(1000);
  oled.clearDisplay();                                                                                                                      // Clear the display
  oled.setTextSize(3);
  oled.setTextColor(SSD1306_WHITE);
  oled.display();

  // BME280 ------------------------------------------------------------------------------------------------------------------------------------------------
  Wire.begin(SDA_BME, SCL_BME);                                                                                                             // Initialize I2C for the BME280. SDA, SCL
  if(!bme.begin(0x76)){
    Serial.println(F("No encuentro un sensor BME280 valido!"));
    while (1);
  }

  // HMC5883 -----------------------------------------------------------------------------------------------------------------------------------------------
  if(!mag.begin()){
    Serial.println(F("No encuentro un sensor HMC5883L valido!"));
    while(1);
  }

  // =======================================================================================================================================================

  client.setServer(mqtt_server, 1883);                                                                                                      // Funcion de la libreria PubSubClient para establecer el servidor MQTT y su puerto
  client.setCallback(callback);                                                                                                             // Funcion de la liberia PubSubClient para definir la funcion de los callbacks

  // Create the queues
  sensorQueue = xQueueCreate(12, sizeof(sensorData));                                                                                       // Queue can hold up to 12 sensor readings
  joystickQueue = xQueueCreate(2, sizeof(joystickData));                                                                                    // Queue can hold up to 2 joystick axis

  // Create tasks and assign them to different cores
  xTaskCreatePinnedToCore(
    mqttTask,                                                                                                                               /* Function to implement the task */
    "mqttTask",                                                                                                                             /* Name of the task */
    10000,                                                                                                                                  /* Stack size in bytes */
    NULL,                                                                                                                                   /* Task input parameter */
    1,                                                                                                                                      /* Priority of the task */
    &mqttTaskHandle,                                                                                                                        /* Task handle. */
    1                                                                                                                                       /* Core where the task should run */
  );

  xTaskCreatePinnedToCore(
    sensorTask,                                                                                                                             /* Function to implement the task */
    "sensorTask",                                                                                                                           /* Name of the task */
    10000,                                                                                                                                  /* Stack size in bytes */
    NULL,                                                                                                                                   /* Task input parameter */
    1,                                                                                                                                      /* Priority of the task */
    &sensorTaskHandle,                                                                                                                      /* Task handle. */
    0                                                                                                                                       /* Core where the task should run */
  );

  xTaskCreatePinnedToCore(
    actuatorTask,                                                                                                                           /* Function to implement the task */
    "actuatorTask",                                                                                                                         /* Name of the task */
    10000,                                                                                                                                  /* Stack size in bytes */
    NULL,                                                                                                                                   /* Task input parameter */
    1,                                                                                                                                      /* Priority of the task */
    &actuatorTaskHandle,                                                                                                                    /* Task handle. */
    0                                                                                                                                       /* Core where the task should run */
  );

  // ---------------------------------------------------------------------------------------------------------------------------------------------------------
  // CONFIGURACION DEL HOTSPOT QUE CREA EL ESP. 3 MODOS: NOMBRE AUTOMATICO CON ID DEL ESP, NOMBRE A ELEGIR Y NOMBRE A ELEGIR CON CONTRASEÑA A ELEGIR
  // ---------------------------------------------------------------------------------------------------------------------------------------------------------
  WiFiManager wm;                                                                                                                           // WiFiManager inicializacion local

  bool res;                                                                                                                                 // Variable de la libreria WiFiManager para comprobar el estado de conexion a la red WiFi

  res = wm.autoConnect("AP_Moya","m3di4l4b");                                                                                               // Modo del hotspot con nombre y contraseña personalizados

  if(!res) {
    Serial.println(F("Incapaz de conectarse a la WiFi elegida"));
  } 
  else {                                                                                                                                    // Si 'res' devuelve un true, es que la conexion es exitosa
    Serial.println(F("¡Conectado a la WiFi elegida!"));
  }
  // ---------------------------------------------------------------------------------------------------------------------------------------------------------

  Serial.println(F("Temporizador establecido para 5s, la primera medida se tomara tras dicho tiempo"));
}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------

void loop(){
  delay(1000);
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION CALLBACK - FUNCION A LA QUE LLEGAN LOS MENSAJES DE LOS TOPICOS SUSCRITOS PARA HACER CAMBIOS EN EL MICRO, ES COMO UN SEGUNDO LOOP
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void callback(char* topic, byte* message, unsigned int length){                                                                             // Funcion que recibe el topico MQTT, el mensaje y la longitud del mismo
  String messageSub;
  
  for(int i = 0; i < length; i++){                                                                                                          // Bucle para printear el mensaje (caracter a caracter)
    Serial.print((char)message[i]);
    messageSub += (char)message[i];                                                                                                        // En 'messageTemp' se carga el contenido de 'message', el cual se reinicia tras cada iteracion del 'loop'
  }

  Serial.print(F("Message arrived on topic: "));
  Serial.print(topic);                                                                                                                      // En 'topic' se guarda el nombre del topic. Por ejemplo 'moya/luces'
  Serial.print(F(". Message: "));
  Serial.println();

  // ===================================================================================================================================================
  // Feel free to add more if statements to control more GPIOs with MQTT
  // ===================================================================================================================================================
  if(String(topic) == "moya/luces"){                                                                                                        // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
    Serial.print(F("Changing output to "));                                                                                                    // Cambien el estado del output dependiendo del mensaje enviado en el topico de suscripcion
    if(messageSub == "on"){
      ledStateON = true; Serial.println(F("on LED"));
    }else if(messageSub == "off"){
      ledStateOFF = true; Serial.println(F("off LED"));
    }
  }

  if(String(topic) == "moya/claxon"){
    if(messageSub == "moc"){
      claxonState = true; Serial.println(F("on Claxon"));
    }
  }

  if(String(topic) == "moya/oled"){
    if(messageSub == "onOLED"){
      oledStateON = true; Serial.println(F("on OLED"));
    }else if(messageSub == "offOLED"){                                                                                                      // Si se recibe un OFF
      oledStateOFF = false; Serial.println(F("off OLED"));
    }
  }

  // Control por joystick de NodeRED UI para los servos ----------------------------------------------------------------------------
  joystickData joyData = {potXaxis, potYaxis};
  if(String(topic) == "moya/xAxis"){
    joyData.potXaxis = messageSub.toInt();                                                                         // Funcion para recibir el valor del eje del joystick en formato string el valor numerico, se transforma el string con el numero en un numero entero con 'toInt()'
  }else if(String(topic) == "moya/yAxis")
    joyData.potYaxis = messageSub.toInt();
  }

  xQueueSend(joystickQueue, &joyData, portMAX_DELAY);
  // -------------------------------------------------------------------------------------------------------------------------------
  // ===================================================================================================================================================

}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION RECONNECT - FUNCION QUE ESTABLECE LA CONNEXION POR MQTT PARA RECIBIR LOS MENSAJES DE LOS TOPICOS A LOS QUE SE ESTA SUSCRITO
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void reconnect(){
  while(!client.connected()){                                                                                                               // Itera hasta reconectar, O SIMPLEMENTE CONECTAR
    Serial.print(F("Attempting MQTT connection..."));
    if (client.connect("ESP32_Moya")) {                                                                                                     // Conexion establecida con el dispositivo MQTT. CUIDADO QUE AQUI SE PUEDE AÑADIR ,"USER", "PASSWORD"
      Serial.println(F("connected"));

      // ===================================================================================================================================================
      // Feel free to add more if statements to control more GPIOs with MQTT
      // ===================================================================================================================================================
      client.subscribe("moya/luces");
      client.subscribe("moya/claxon");
      client.subscribe("moya/xAxis");
      client.subscribe("moya/yAxis");
      client.subscribe("moya/oled");
      // ===================================================================================================================================================

    }else{
      Serial.print(F("failed, rc="));                                                                                                          // Si no se establece conexion
      Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));                                                                                            // Espera 5 segundos hasta reintentar la conexion
      delay(5000);
    }
  }
}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------

float current_heading(){
  sensors_event_t event;
  mag.getEvent(&event);
  
  float head = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.007;
  head += declinationAngle;

  if(head < 0) head += 2*PI;                                                                                                                // Correct for when signs are reversed
  if(head > 2*PI) head -= 2*PI;                                                                                                             // Check for wrap due to addition of declination

  return head * 180 / M_PI;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION UPDATE OLED DISPLAY - FUNCION QUE REDIBUJA LA INFORMACION EN EL OLED
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void updateOledDisplay(){
  oled.clearDisplay();                                                                                                                      // Se limpia el buffer del OLED
  oled.setTextSize(1);                                                                                                                      // Se selecciona el tamaño de la letra

  oled.setCursor(10,5);                                                                                                                     // Se indica el pixel donde se quiere escribir (X,Y)
  oled.print("Temp: ");                                                                                                                     // Se escribe un string
  oled.setCursor(40,5);
  oled.print(bme.readTemperature());                                                                                                        // Se escribe el valor actual de la temperatura del BME280
  oled.setCursor(75,5);
  oled.print((char)247);
  oled.setCursor(81,5);
  oled.println("C");

  oled.setCursor(10,17);
  oled.print("Pres: ");
  oled.setCursor(40,17);
  oled.print(bme.readPressure() / 100.0F);
  oled.setCursor(80,17);
  oled.print(" hPa");

  oled.setCursor(10,29);
  oled.print("Alti: ");
  oled.setCursor(40,29);
  oled.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  oled.setCursor(70,29);
  oled.print(" m");

  oled.setCursor(10,41);
  oled.print("Hume: ");
  oled.setCursor(40,41);
  oled.print(bme.readHumidity());
  oled.setCursor(70,41);
  oled.print(" %");

  oled.setCursor(10,53);
  oled.print("Bate: ");
  oled.setCursor(40,53);
  oled.print((float)(analogRead(VBAT_PIN)) / 4095*2*3.3*1.1);                                                                               // Calculo matematico siguiendo el circuito de lectura de bateria de LilyGO para expresar el voltaje en el conector de bateria
  oled.setCursor(65,53);
  oled.print(" V");

  oled.display();
}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
