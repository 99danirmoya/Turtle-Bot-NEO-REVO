/* ***********************************************************************************************************************************************************
TURTLE BOT NEO EVO (PUBLISH/SUBSCRIBE MQTT): en este skecth sirve para programar un robot de dos ruedas que cuenta con un sensor climatico BME280 que publica
las medidas cada 5 segundos en el servidor MQTT https://emqx.broker.io para ser procesadas en NodeRED. Además, en NodeRED se crean los topicos necesarios
para controlar la pantalla OLED, el LED, los servos de rotacion continua y el zumbador a bordo del robot, que reciben los parametros de un dashboard de
NodeRED por medio de estar suscrito a los topicos donde se envian dichos parametros.
*********************************************************************************************************************************************************** */

// ===========================================================================================================================================================
// INLCUSION DE LIBRERIAS
// ===========================================================================================================================================================
#include <Arduino.h>
#include <WiFiManager.h>                                                                                  // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
// ===========================================================================================================================================================

// ===========================================================================================================================================================
// MACROS (de ser necesarias)
// ===========================================================================================================================================================
#define OLED_ADDR       0x3C
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1

#define LED_PIN         40
#define PIN_SERVO_RIGHT 45
#define PIN_SERVO_LEFT  41
#define SDA_BME         38
#define SCL_BME         39
#define SDA_OLED        18
#define SCL_OLED        17
#define VBAT_PIN        1
#define BUZZER_PIN      46

#define SEALEVELPRESSURE_HPA (1015)
// ===========================================================================================================================================================

// ===========================================================================================================================================================
// CONSTRUCTORES DE OBJETOS DE CLASE DE LIBRERIA, VARIABLES GLOBALES, CONSTANTES...
// ===========================================================================================================================================================
const char* mqtt_server = "broker.emqx.io";
WiFiClient espClient;                                                                                     // Objeto de la libreria WiFiManager
PubSubClient client(espClient);                                                                           // Objeto de la libreria MQTT

Adafruit_BME280 bme;                                                                                      // Objeto del sensor BME280

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);                                   // Objeto del OLED

Servo miServoRight;                                                                                       // Objeto del servo derecho
Servo miServoLeft;                                                                                        // Objeto del servo izquierdo

float temp, hum, alt, pres, vbat;                                                                         // Definicion de las variables del sensor BME280

int potXaxis, potYaxis;                                                                                   // Variables donde se guarda el valor de los ejes del potenciometro de NodeRED Dashboard

unsigned long lastTime = 0;                                                                               // Se inicializa la variable en la que se guarda el tiempo en milisegundos tras cada iteracion del loop
const unsigned long timerDelay = 5000;                                                                    // Se inicializa el intervalo de publicacion de datos

bool oledState = false;                                                                                   // Flag to track OLED state
unsigned long lastOledUpdateTime = 0;                                                                     // Timer for OLED updates

const int ledChannel  = 0;                                                                                // Canal del LED que se controla por PWM
const int frequency = 5000;                                                                               // Frecuencia de la señal PWM
const int resolution  = 8;                                                                                // Resolucion del PWM

const int duration = 500;                                                                                 // Duracion de las notas del buzzer en ms
// ===========================================================================================================================================================

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION SETUP - SOLO SE EJECUTA UNA VEZ
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // =======================================================================================================================================================
  // INICIALIZACION DE I/O
  // =======================================================================================================================================================
  pinMode(BUZZER_PIN, OUTPUT);                                                                            // El zumbador es un output digital

  ledcAttach(LED_PIN, frequency, resolution);                                                             // Inicializacion del PWM para el LED
  miServoRight.attach(PIN_SERVO_RIGHT);                                                                   // Union del servo al pin correspondiente
  miServoLeft.attach(PIN_SERVO_LEFT);

  Wire.begin(SDA_BME, SCL_BME);                                                                           // Initialize I2C for the BME280. SDA, SCL
  Wire1.begin(SDA_OLED, SCL_OLED);                                                                        // Initialize I2C for the OLED. SDA, SCL

  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.display();                                                                                         // Display initialization - LOGO ADAFRUIT
  delay(1000);
  oled.clearDisplay();                                                                                    // Clear the display
  oled.setTextSize(3);
  oled.setTextColor(SSD1306_WHITE);
  oled.display();

  if (!bme.begin(0x76)){
    Serial.println("No encuentro un sensor BME280 valido!");
    while (1);
  }
  // =======================================================================================================================================================

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // ---------------------------------------------------------------------------------------------------------------------------------------------------------
  // CONFIGURACION DEL HOTSPOT QUE CREA EL ESP. 3 MODOS: NOMBRE AUTOMATICO CON ID DEL ESP, NOMBRE A ELEGIR Y NOMBRE A ELEGIR CON CONTRASEÑA A ELEGIR
  // ---------------------------------------------------------------------------------------------------------------------------------------------------------
  WiFiManager wm;                                                                                         //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around

  bool res;

  res = wm.autoConnect("AP_Moya","m3di4l4b");                                                             // password protected ap

  if(!res) {
    Serial.println("Failed to connect");
  } 
  else { 
    Serial.println("connected...yeey :)");                                                              // if you get here you have connected to the WiFi   
  }
  // ---------------------------------------------------------------------------------------------------------------------------------------------------------

  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION LOOP - TRAS HABERSE COMPLETADO LA CONFIGURACION INICIAL, ESTE ES EL ALGORITMO PARA EL MQTT PARA EL PUBLISHING
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  if (!client.connected()) {                                                                              // Si no hay conexion
    reconnect();                                                                                          // Entra la funcion de reconexion
  }
  client.loop();

  if ((millis() - lastTime) > timerDelay) {                                                               // Send an MQTT publish request every 5 secs
    if(WiFi.status()== WL_CONNECTED){                                                                     // Check WiFi connection status
      // ===================================================================================================================================================
      // PREPARACION DE LAS MEDICIONES DEL SENSOR Y CONVERSION A STRING DE CARACTERES PARA ENVIARSE POR MQTT
      // =====================================================================================================================================================
      char dataStr[40];                                                                                   // Se crea un string de caracteres para guardar ambas medidas, se reservan 20 espacios
      temp = bme.readTemperature();                                                                       // Variable que guarda la temperatura del sensor BME280
      pres = bme.readPressure() / 100.0F;                                                                 // Variable que guarda la presion del sensor BME280
      alt = bme.readAltitude(SEALEVELPRESSURE_HPA);                                                       // Variable que guarda la altitud del sensor BME280
      hum = bme.readHumidity();                                                                           // Variable que guarda la humedad del sensor BME280
      vbat = (float)(analogRead(VBAT_PIN)) / 4095*2*3.3*1.1;                                              // Variable que guarda el valor del voltaje en el conector de batería

      sprintf(dataStr, "%5.2f, %6.2f, %4.2f, %4.2f, %4.3f", temp, pres, alt, hum, vbat);                  // La funcion 'sprintf' de C++ se usa para poder introducir las medidas y elegir el ancho de numero y su precision. Las medidas se separan con una coma ','
      Serial.println(dataStr);                                                                            // Muestra en el serial de Arduino el string

      client.publish("moya/sensores", dataStr);                                                           // Se publica el string con los datos de los sensores en el topico 'moya/sensores'
      // =====================================================================================================================================================

    }else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();                                                                                  // Se refresca el valor de 'lastTime' al tiempo actual tras enviar el 'dataStr'
  }
  
  if (oledState && (millis() - lastOledUpdateTime >= 5000)) {                                             // Update OLED display if it is turned on and 5 seconds have passed
    updateOledDisplay();
    lastOledUpdateTime = millis();
  }
}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION CALLBACK - FUNCION A LA QUE LLEGAN LOS MENSAJES DE LOS TOPICOS SUSCRITOS PARA HACER CAMBIOS EN EL MICRO, ES COMO UN SEGUNDO LOOP
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void callback(char* topic, byte* message, unsigned int length) {                                          // Funcion que recibe el topico MQTT, el mensaje y la longitud del mismo
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);                                                                                    // En 'topic' se guarda el nombre del topic. Por ejemplo 'moya/luces'
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {                                                                      // Bucle para printear el mensaje (caracter a caracter)
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];                                                                      // En 'messageTemp' se carga el contenido de 'message', el cual se reinicia tras cada iteracion del 'loop'
  }
  Serial.println();

  // ===================================================================================================================================================
  // Feel free to add more if statements to control more GPIOs with MQTT
  // ===================================================================================================================================================
  if(String(topic) == "moya/luces"){                                                                      // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
    Serial.print("Changing output to ");                                                                  // Changes the output state according to the message
    if(messageTemp == "on"){
      Serial.println("on LED");
      for(int i = 0; i <= 255; i += 1){
        ledcWrite(LED_PIN,i);
        delay(1);
      }
    }
    else if(messageTemp == "off"){
      Serial.println("off LED");
      for(int i = 255; i >= 0; i-= 1){
        ledcWrite(LED_PIN,i);
        delay(1);
      }
    }
  }

  if(String(topic) == "moya/claxon"){
    if(messageTemp == "moc"){
      Serial.println("on Claxon");
      tone(BUZZER_PIN, 300, duration);
    }
  }

  // Control por joystick de NodeRED UI para los servos ----------------------------------------------------------------------------
  if(String(topic) == "moya/xAxis") potXaxis = messageTemp.toInt();                                        // Funcion para recibir el valor del eje del joystick en formato string el valor numerico, se transforma el string con el numero en un numero entero con 'toInt()'
  if(String(topic) == "moya/yAxis") potYaxis = messageTemp.toInt();

  int servoXPosition = map(potXaxis, -100, 100, 45, -45);                                                 // Lo mismo para el eje X del joystick es al revés, 0 es -45º y 1023, 45º. SE USA 45 PORQUE 90 PRODUCE ESPASMOS EN LOS MOTORES
  int servoYPosition = map(potYaxis, -100, 100, -45, 45);                                                 // Posición neutra del eje Y del joystick es 0, que corresponde a 0º en el servo. Respectivamente, -100 es 45º y 100, -45º. Devuelve valores de -45 a 45

  miServoRight.write(90 + servoYPosition + servoXPosition);                                               // 90 - 0 - 45 = 45 -> Hacia atrás PORQUE ESTÁ PUESTO "ESPEJO" RESPECTO DEL OTRO EN EL ROBOT
  miServoLeft.write(90 - servoYPosition + servoXPosition);                                                // 90 + 0 - 45 = 45 -> Hacia adelante
  // -------------------------------------------------------------------------------------------------------------------------------

  if(String(topic) == "moya/oled"){
    if(messageTemp == "onOLED"){
      Serial.println("on OLED");    
      
      oledState = true;
      updateOledDisplay();
      lastOledUpdateTime = millis();                                                                      // Actualizar timer
    }
    else if(messageTemp == "offOLED"){                                                                    // Si se recibe un OFF
      Serial.println("off OLED");
      
      oledState = false;
      
      oled.clearDisplay();                                                                                // Se limpia el buffer del OLED
      oled.display();                                                                                     // Se printea el buffer que, como no es nada, se apaga
    }
  }
  // ===================================================================================================================================================

}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------

void updateOledDisplay() {
  oled.clearDisplay();                                                                                // Se limpia el buffer del OLED
  oled.setTextSize(1);                                                                                // Se selecciona el tamaño de la letra

  oled.setCursor(10,5);                                                                               // Se indica el pixel donde se quiere escribir (X,Y)
  oled.print("Temp: ");                                                                               // Se escribe un string
  oled.setCursor(40,5);
  oled.print(bme.readTemperature());                                                                  // Se escribe el valor actual de la temperatura del BME280
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
  oled.print((float)(analogRead(VBAT_PIN)) / 4095*2*3.3*1.1);
  oled.setCursor(65,53);
  oled.print(" V");

  oled.display();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCION RECONNECT - FUNCION QUE ESTABLECE LA CONNEXION POR MQTT PARA RECIBIR LOS MENSAJES DE LOS TOPICOS A LOS QUE SE ESTA SUSCRITO
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
void reconnect() {
  while (!client.connected()) {                                                                           // Loop until we're reconnected (OR JUST CONNECTED)
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32_Moya")) {                                                                   // Conexion establecida con el dispositivo MQTT. CUIDADO QUE AQUI SE PUEDE AÑADIR ,"USER", "PASSWORD"
      Serial.println("connected");

      // ===================================================================================================================================================
      // Feel free to add more if statements to control more GPIOs with MQTT
      // ===================================================================================================================================================
      client.subscribe("moya/luces");
      client.subscribe("moya/claxon");
      client.subscribe("moya/xAxis");
      client.subscribe("moya/yAxis");
      client.subscribe("moya/oled");
      // ===================================================================================================================================================

    } else {
      Serial.print("failed, rc=");                                                                        // Si no se establece conexion
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");                                                          // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
// -----------------------------------------------------------------------------------------------------------------------------------------------------------
