// MACROS ------------------------------------------------------------------------
// Servos
#define LEFT_SERVO_PIN  18
#define RIGHT_SERVO_PIN 19

// GPS
#define SerialGPS       Serial1                                                   // Se crea un nuevo puerto serie para el GPS (el predeterminado, 'Serial', se usa para el monitor serie)
#define GPS_RX_PIN      44                                                        // Pin RX del ESP32 al pin TX del GPS
#define GPS_TX_PIN      43                                                        // Pin TX del ESP32 al pin RX del GPS
#define GPS_BAUD_RATE   9600                                                      // Baudios para la comunicacion serie del GPS

// I2C
#define SDA_EXT         38
#define SCL_EXT         39

// Conversions
#define METERS_PER_DEGREE_LAT 111000                                              // Approximation of distance (meters per degree): 111,000 meters/degree latitude