#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <DHT.h>
#include <DHT_U.h>

//#define DEBUG

#define DHT11_PIN     12
#define PLUMINICA_PIN A5
#define VOLTAGE_PIN   A4
#define PIN_LED       13

    #define FONA_RX  9
    #define FONA_TX  8
    #define FONA_RST 4
    #define FONA_RI  7

#define FONA_APN             "internet.ideasclaro"              // APN usado por el ISP telefonico

#define MAX_TX_FAILURES      5                      // Cantidad maxima de intentos de push

const char SERVER[] = "http://io.vbalex.com:8081/";
const char usuario[] = "cablevision";
const char api_key[] = "6f7264657265645f757569642855554944282929";
const char  id[] = "3666373236343635373236353634356637353735";

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);                 // Comunicacion serial con FONA(Modulo GSM/GPRS).
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);       // Coneccion de la biblioteca FONA.


Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
DHT_Unified dht(DHT11_PIN, DHT11);


const String STR_VALUE_SEPARATOR = ":";
const String STR_GROUP_SEPARATOR = ";";

uint8_t txFailures = 0;

void setup() {
  Serial.begin(9600);
  //while (!Serial);
  Watchdog.enable(8 * 1000);// es lo maximo que se puede en avr
  Watchdog.reset();
  fonaSerial->begin(4800);
  if (!fona.begin(*fonaSerial)) {
    halt(F("Couldn't find FONA"));
  }
  Watchdog.reset();
 if (!accel.begin()) {
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
  }
  Watchdog.reset();
  if (!gyro.begin()) {
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
  }
  Watchdog.reset();
  dht.begin();
  pinMode(PLUMINICA_PIN, INPUT);
  pinMode(PIN_LED, OUTPUT);
  Watchdog.reset();
  fonaSerial->println(F("AT+CMEE=2"));
  Watchdog.reset();
  Serial.println(F("FONA is OK"));
  delay(5*1000);
  Watchdog.reset();
  delay(5*1000);
  Watchdog.reset();
  delay(5*1000);
  Watchdog.reset();
  delay(5*1000);
  Watchdog.reset();
  
  // Wait for FONA to connect to cell network (up to 8 seconds, then watchdog reset).
  Serial.println(F("Checking for network..."));
  while (fona.getNetworkStatus() != 1) {
    delay(250);
  }
  Watchdog.reset();
  //Enable for manual APN
  fona.setGPRSNetworkSettings(F(FONA_APN));
  delay(2000);
  Watchdog.reset();
  Serial.println(F("Disabling GPRS"));
  fona.enableGPRS(false);
  Watchdog.reset();
  delay(5000);
  Watchdog.reset();
  Serial.println(F("Enabling GPRS"));
  if (!fona.enableGPRS(true)) {
    halt(F("Failed to turn GPRS on, resetting..."));
  }
  Serial.println(F("Connected to Cellular!"));
  // Wait a little bit to stabilize the connection.
  Watchdog.reset();
  delay(2500);
  Serial1.begin(9600);
}


sensors_event_t event;
String str;
void loop() {
  if (txFailures >= MAX_TX_FAILURES) {
    halt(F("Connection lost, resetting..."));
  }

  while (Serial1.available()) {
    Serial1.read();
    delay(5);
  }
  Watchdog.reset();
  bool found = false;
  while (!found) {
    if (Serial1.available()) {
      str = Serial1.readStringUntil('$');
      if (str.indexOf("GPRMC") != -1) {
        logString(str, F("gps"));
        found = true;
      }
    }
  }
  Watchdog.reset();
  uint16_t v;
  fona.getBattPercent(&v);
  logInteger(v, "bateria");
  Watchdog.reset();
  
  unsigned long t=millis();
  while(millis()-t<50){
    int sensorValue = analogRead(VOLTAGE_PIN);
    if(v<sensorValue){
      v = sensorValue;
    }
  }
  logInteger(v*118.5/1023.0,"voltage");
  
  uint32_t potenciaLuminica = (analogRead(PLUMINICA_PIN) / 1023.0 * 3330); //milivoltios
  logInteger(potenciaLuminica, "potenciaLuminica");
  Watchdog.reset();

  accel.getEvent(&event);
  str = event.acceleration.x + STR_VALUE_SEPARATOR + event.acceleration.y + STR_VALUE_SEPARATOR + event.acceleration.z;
  gyro.getEvent(&event);
  str += STR_GROUP_SEPARATOR;
  str += event.acceleration.x + STR_VALUE_SEPARATOR + event.acceleration.y + STR_VALUE_SEPARATOR + event.acceleration.z;
  logString(str, F("movimiento"));
  Watchdog.reset();

  dht.temperature().getEvent(&event);
  str = event.temperature;
  str += STR_VALUE_SEPARATOR;
  dht.humidity().getEvent(&event);
  str += event.relative_humidity;
  logString(str, F("condiciones"));
  Watchdog.reset();

}

void logInteger(uint32_t indicador, const String &nombre) {
#ifdef DEBUG
  Serial.print(nombre);
  Serial.print(":   ");
  Serial.println(indicador);
#endif
  String str = "usuario=";
  str += usuario;
  str += F("&key=");
  str += api_key;
  str += F("&dispositivo=");
  str += id;
  str += F("&nombre=");
  str += nombre;
  str += F("&valor=");
  str += indicador;
  int code = post(str.c_str());
  if (code != 200) {
    txFailures++;
  } else {
    txFailures = 0;
  }
}

void logString(const String &cadena, const String &nombre) {
#ifdef DEBUG
  Serial.print(nombre);
  Serial.print(":   ");
  Serial.println(cadena);
#endif
  String str = "usuario=";
  str += usuario;
  str += F("&key=");
  str += api_key;
  str += F("&dispositivo=");
  str += id;
  str += F("&nombre=");
  str += nombre;
  str += F("&valor=");
  str += cadena;
  int code = post(str.c_str());
  if (code != 200) {
    txFailures++;
  } else {
    txFailures = 0;
  }

}
void halt(const __FlashStringHelper *error) {
  Serial.println(error);
  while (1) {
    digitalWrite(PIN_LED, LOW);
    delay(100);
    digitalWrite(PIN_LED, HIGH);
    delay(100);
  }
}

uint16_t post(const char data[]) {
  uint16_t statuscode;
  int16_t length;
  fona.HTTP_POST_start(SERVER, F("text/plain"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length) ;
  while (length > 0) {
    while (fona.available()) {
      fona.read();
      length--;
      if (! length) break;
    }
  }
  fona.HTTP_POST_end();
  return statuscode;
}

