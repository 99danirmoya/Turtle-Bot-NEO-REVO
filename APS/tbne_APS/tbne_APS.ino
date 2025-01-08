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
#include <ArduinoJson.h>                                                                                                                    // Libreria para parseo de JSON

#include <Wire.h>                                                                                                                           // Libreria del bus I2C

#include <Adafruit_Sensor.h>                                                                                                                // Libreria para usar sensores de Adafruit
#include <Adafruit_BME280.h>                                                                                                                // Libreria del BME280

#include <Adafruit_GFX.h>                                                                                                                   // Libreria complementaria para el panel OLED
#include <Adafruit_SSD1306.h>                                                                                                               // Libreria especifica para el panel OLED de 128x64 pixeles

#include <ESP32Servo.h>                                                                                                                     // Libreria para manejos de servos para placas que usen ESP32

#include <TinyGPS++.h>                                                                                                                      // Libreria para usar el GPS NEO-6M por puerto serie

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
#define BUZZER_PIN      46
#define DURATION        500                                                                                                                 // Duracion de las notas del buzzer en ms

// Macros para temporalizaciones -----------------------------------------------------------------------------------------------------------------------------
#define TIMER_DELAY     5000                                                                                                                // Se inicializa el intervalo de publicacion de datos
// ===========================================================================================================================================================

// ===========================================================================================================================================================
// CONSTRUCTORES DE OBJETOS DE CLASE DE LIBRERIA, VARIABLES GLOBALES, CONSTANTES...
// ===========================================================================================================================================================
const char* mqtt_server = "srv-iot.diatel.upm.es";                                                                                          // Broker MQTT. UPM broker
WiFiClient espClient;                                                                                                                       // Objeto de la libreria WiFiManager
PubSubClient client(espClient);                                                                                                             // Objeto de la libreria MQTT

//HardwareSerial SerialGPS(1);                                                                                                                // Se inicializa un segundo puerto serie para el GPS
TinyGPSPlus gps;                                                                                                                            // Objeto del GPS

SoftwareSerial sds(SDS_RX, SDS_TX);                                                                                                         // Inicializacion del puerto serie virtual para el sensor SDS011

Adafruit_BME280 bme;                                                                                                                        // Objeto del sensor BME280

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);                                                                     // Objeto del OLED

Servo miServoRight;                                                                                                                         // Objeto del servo derecho
Servo miServoLeft;                                                                                                                          // Objeto del servo izquierdo

float temp, hum, alt, pres, vbat, lat, lon, veloc, pm2_5, pm10;                                                                             // Definicion de las variables 'float'

int potXaxis, potYaxis, pm25int, pm10int;                                                                                                   // Definicion de las variables 'int'

uint8_t sat;                                                                                                                                // Variable en la que se guarda el numero de satelites en cobertura del GPS

unsigned long lastTime = 0;                                                                                                                 // Se inicializa la variable en la que se guarda el tiempo en milisegundos tras cada iteracion del loop

bool oledState = false;                                                                                                                     // Flag to track OLED state
unsigned long lastOledUpdateTime = 0;                                                                                                       // Timer for OLED updates
// ===========================================================================================================================================================

// ===========================================================================================================================================================
// FUNCTION FORWARD DECLARATIONS
// ===========================================================================================================================================================
void callback(char*, byte*, unsigned int);
void reconnect();
void updateOledDisplay();
// ===========================================================================================================================================================

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION SETUP - SOLO SE EJECUTA UNA VEZ
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // =======================================================================================================================================================
  // INICIALIZACION DE I/O
  // =======================================================================================================================================================
  // Buzzer ------------------------------------------------------------------------------------------------------------------------------------------------
  pinMode(BUZZER_PIN, OUTPUT);                                                                                                              // El zumbador es un output digital

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
  // =======================================================================================================================================================

  client.setServer(mqtt_server, 1883);                                                                                                      // Funcion de la libreria PubSubClient para establecer el servidor MQTT y su puerto
  client.setCallback(callback);                                                                                                             // Funcion de la liberia PubSubClient para definir la funcion de los callbacks

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

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION LOOP - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO PARA EL MQTT PARA EL PUBLISHING
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void loop(){
  if(!client.connected()){                                                                                                                  // Si no hay conexion
    reconnect();                                                                                                                            // Entra la funcion de reconexion
  }
  client.loop();

  if((millis() - lastTime) > TIMER_DELAY){                                                                                                  // Send an MQTT publish request every 5 secs
    if(WiFi.status()== WL_CONNECTED){                                                                                                       // Check WiFi connection status
      // ===================================================================================================================================================
      // PREPARACION DE LAS MEDICIONES DEL SENSOR Y CONVERSION A STRING DE CARACTERES PARA ENVIARSE POR MQTT
      // =====================================================================================================================================================
      char dataStr[128];                                                                                                                    // Se crea un string de caracteres para guardar ambas medidas, se reservan 60 espacios
      temp = bme.readTemperature();                                                                                                         // Variable que guarda la temperatura del sensor BME280
      pres = bme.readPressure() / 100.0F;                                                                                                   // Variable que guarda la presion del sensor BME280
      alt = bme.readAltitude(SEALEVELPRESSURE_HPA);                                                                                         // Variable que guarda la altitud del sensor BME280
      hum = bme.readHumidity();                                                                                                             // Variable que guarda la humedad del sensor BME280
      vbat = (float)(analogRead(VBAT_PIN)) / 4095*2*3.3*1.1;                                                                                // Variable que guarda el valor del voltaje en el conector de batería

      // Preparacion del GPS para tener cobertura satelite y poder enviar coordenadas ------------------------------------------------------------------------
      while(SerialGPS.available() > 0){                                                                                                     // Mientras haya bytes disponibles en el puerto serie del GPS
        if(gps.encode(SerialGPS.read())){                                                                                                   // Se confirma que haya lecturas del GPS
          if(gps.location.isValid()){                                                                                                       // Si las coordenadas recogidas son validas
            lat = gps.location.lat();                                                                                                       // Se guardan en las variables 'lat' y 'lon' dichas coordenadas
            lon = gps.location.lng();
            sat = gps.satellites.value();
            veloc = gps.speed.kmph();
          }else{                                                                                                                            // En caso de que sean invalidas, se muestra un mensaje en el monitor serie
            lat = 0.0;                                                                                                                      // Por marcar un numero definido, la ubicacion se fija a 0 en ambos 'lat' y 'lon'
            lon = 0.0;
            sat = 0;
            veloc = 0.0;
            Serial.println(F("INVALID GPS"));
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
          pm2_5 = pm25int / 10.0;
          pm10 = pm10int / 10.0;
        }else{
          Serial.println(F("Invalid ending byte from SDS011"));
          pm2_5 = 0.0;
          pm10 = 0.0;
        }
      }else{
        Serial.println(F("Not enough data from SDS011"));
      }

      if(millis() > 15000 && gps.charsProcessed() < 10){                                                                                    // Si durante 15 segundos se cumple que los caracteres procesados del GPS son menos de 10, implica que hay un error leyendo el GPS, se reporta el fallo y se bloquea el programa hasta que se solvente el error
        Serial.println(F("GPS no detectado: comprueba el cableado"));
        delay(15000);
      }
      // -----------------------------------------------------------------------------------------------------------------------------------------------------

      snprintf(dataStr, sizeof(dataStr), "{\"temperature\":%5.2f,\"pressure\":%6.2f,\"altitude\":%4.2f,\"humidity\":%4.2f,\"battery\":%4.3f,\"latitude\":%10.6f,\"longitude\":%10.6f,\"sats\":%1d,\"speed\":%5.2f,\"PM2.5\":%6.2f,\"PM10\":%6.2f}", temp, pres, alt, hum, vbat, lat, lon, sat, veloc, pm2_5, pm10);  // La funcion 'sprintf' de C++ se usa para poder introducir las medidas y elegir el ancho de numero y su precision. Las medidas se separan con una coma ','
      Serial.println(dataStr);                                                                                                              // Muestra en el serial de Arduino el string

      client.publish("moya/sensores", dataStr);                                                                                             // Se publica el string con los datos de los sensores en el topico 'moya/sensores'
      // =====================================================================================================================================================

    }else{
      Serial.println(F("WiFi Disconnected"));
    }
    lastTime = millis();                                                                                                                    // Se refresca el valor de 'lastTime' al tiempo actual tras enviar el 'dataStr'
  }
  
  if(oledState && (millis() - lastOledUpdateTime >= 5000)){                                                                                 // Actualiza el OLED si esta encendido y han pasado 5 segundos
    updateOledDisplay();
    lastOledUpdateTime = millis();
  }
}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION CALLBACK - FUNCION A LA QUE LLEGAN LOS MENSAJES DE LOS TOPICOS SUSCRITOS PARA HACER CAMBIOS EN EL MICRO, ES COMO UN SEGUNDO LOOP
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void callback(char* topic, byte* message, unsigned int length){                                                                             // Funcion que recibe el topico MQTT, el mensaje y la longitud del mismo
  Serial.print(F("Message arrived on topic: "));
  Serial.print(topic);                                                                                                                      // En 'topic' se guarda el nombre del topic. Por ejemplo 'moya/luces'
  Serial.print(F(". Message: "));
  String messageTemp;
  
  for(int i = 0; i < length; i++){                                                                                                          // Bucle para printear el mensaje (caracter a caracter)
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];                                                                                                        // En 'messageTemp' se carga el contenido de 'message', el cual se reinicia tras cada iteracion del 'loop'
  }
  Serial.println();

  StaticJsonDocument<256> doc;                                                                                                              // Objeto en el que almacenar el JSON recibido
  DeserializationError error = deserializeJson(doc, messageTemp);                                                                           // JSON error handling
  if(error){
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }

  // ===================================================================================================================================================
  // Feel free to add more if statements to control more GPIOs with MQTT
  // ===================================================================================================================================================
  if(doc.containsKey("luces")){                                                                                                        // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
    String luces = doc["luces"].as<String>();
    Serial.print(F("Changing output to "));                                                                                                    // Cambien el estado del output dependiendo del mensaje enviado en el topico de suscripcion
    if(luces == "on"){
      Serial.println(F("on LED"));
      for(int i = 0; i <= 255; i += 1){
        ledcWrite(LED_PIN,i);
        delay(1);
      }
    }
    else if(luces == "off"){
      Serial.println(F("off LED"));
      for(int i = 255; i >= 0; i-= 1){
        ledcWrite(LED_PIN,i);
        delay(1);
      }
    }
  }

  if(doc.containsKey("claxon")){
    String claxon = doc["claxon"].as<String>();
    if(claxon == "moc"){
      Serial.println(F("on Claxon"));
      tone(BUZZER_PIN, 300, DURATION);
    }
  }

  // Control por joystick de NodeRED UI para los servos ----------------------------------------------------------------------------
  if(doc.containsKey("xAxis")) potXaxis = doc["xAxis"].as<int>();                                                                         // Funcion para recibir el valor del eje del joystick en formato string el valor numerico, se transforma el string con el numero en un numero entero con 'toInt()'
  if(doc.containsKey("yAxis")) potYaxis = doc["yAxis"].as<int>();
  Serial.print(F("Eje X: ")); Serial.print(potXaxis); Serial.print(F("\tEje Y: ")); Serial.println(potYaxis);

  int servoXPosition = map(potXaxis, -100, 100, 45, -45);                                                                                   // Posición neutra del eje X del joystick es 0, que corresponde a 0º en el servo. Respectivamente, -100 es 45º y 100, -45º. Devuelve valores de -45 a 45
  int servoYPosition = map(potYaxis, -100, 100, -45, 45);                                                                                   // Lo mismo para el eje Y del joystick es al revés, -100 es -45º y 100, 45º

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
  // -------------------------------------------------------------------------------------------------------------------------------

  if(doc.containsKey("oled")){
    String oledCommand = doc["oled"].as<String>();
    if(oledCommand == "onOLED"){
      Serial.println(F("on OLED"));    
      
      oledState = true;
      updateOledDisplay();
      lastOledUpdateTime = millis();                                                                                                        // Actualizar timer
    }
    else if(oledCommand == "offOLED"){                                                                                                      // Si se recibe un OFF
      Serial.println(F("off OLED"));
      
      oledState = false;
      
      oled.clearDisplay();                                                                                                                  // Se limpia el buffer del OLED
      oled.display();                                                                                                                       // Se printea el buffer que, como no es nada, se apaga
    }
  }
  // ===================================================================================================================================================

}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION RECONNECT - FUNCION QUE ESTABLECE LA CONNEXION POR MQTT PARA RECIBIR LOS MENSAJES DE LOS TOPICOS A LOS QUE SE ESTA SUSCRITO
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void reconnect(){
  while(!client.connected()){                                                                                                               // Itera hasta reconectar, O SIMPLEMENTE CONECTAR
    Serial.print(F("Attempting MQTT connection..."));
    if (client.connect("hSQ6oIHn5dJsslmQEdsk")) {                                                                                           // Conexion establecida con el dispositivo MQTT. CUIDADO QUE AQUI SE PUEDE AÑADIR ,"USER", "PASSWORD" (UPM THINGSBOARD DEVICE TOKEN)
      Serial.println(F("connected"));

      // ===================================================================================================================================================
      // Feel free to add more if statements to control more GPIOs with MQTT
      // ===================================================================================================================================================
      client.subscribe("moya/actuators");
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
