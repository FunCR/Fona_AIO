install #include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_FONA.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>


const int ledPin = 13;

#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7
#define DHT_PIN 12

#define FONA_APN             "kolbi3g"  // APN used by cell data service 

#define AIO_SERVER           "io.adafruit.com"      // Adafruit IO server name.
#define AIO_SERVERPORT       1883                   // Adafruit IO port.
#define AIO_USERNAME         "alexvargasbenamburg"  // Adafruit IO username .
#define AIO_KEY              "5fb35742021b47ca8695e29c392fe051"  // Adafruit IO key.

#define MAX_TX_FAILURES      3  // Maximum number of publish failures in a row before resetting the whole sketch.
#define GPSECHO  true

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);     // FONA software serial connection.
SoftwareSerial gpsSS(10, 11);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);                 // FONA library connection.
Adafruit_GPS GPS(&gpsSS);
DHT_Unified dht(DHT_PIN, DHT11);

Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER,AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

uint8_t txFailures = 0;                                       // Count of how many publish failures have occured in a row.
uint8_t usingInterrupt = false;

const char BATTERY_FEED[] = AIO_USERNAME "/feeds/battery";
const char LOCATION_FEED[] = AIO_USERNAME "/feeds/location";
const char HUMIDITY_FEED[] = AIO_USERNAME "/feeds/humidity";
const char TEMPERATURE_FEED[] = AIO_USERNAME "/feeds/temperature";

Adafruit_MQTT_Publish battery_feed = Adafruit_MQTT_Publish(&mqtt, BATTERY_FEED);
Adafruit_MQTT_Publish location_feed = Adafruit_MQTT_Publish(&mqtt, LOCATION_FEED);
Adafruit_MQTT_Publish humidity_feed = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);
Adafruit_MQTT_Publish temperature_feed = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);

void setup() {
  
  // Initialize serial output.
  Serial.begin(115200);
  // Set alarm components
  pinMode(ledPin, OUTPUT);
  dht.begin();
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
  gpsSS.println(PMTK_Q_RELEASE);
  
  // Initialize the FONA module
  Serial.println(F("Initializing FONA....(may take 10 seconds)"));
  fonaSS.begin(4800);
  if (!fona.begin(fonaSS)) {
    halt(F("Couldn't find FONA"));
  }
  
  fonaSS.println("AT+CMEE=2");
  Serial.println(F("FONA is OK"));
  
  Watchdog.enable(15000);
  Watchdog.reset();  
  // Wait for FONA to connect to cell network (up to 8 seconds, then watchdog reset).
  Serial.println(F("Checking for network..."));
  while (fona.getNetworkStatus() != 1) {
   delay(500);
  }
  Watchdog.reset();
  //Enable for manual APN
  fona.setGPRSNetworkSettings(F(FONA_APN));
  delay(2000);
  Watchdog.reset();  
  Serial.println(F("Disabling GPRS"));
  fona.enableGPRS(false);
  delay(2000);
  Watchdog.reset();  
  Serial.println(F("Enabling GPRS"));
  if (!fona.enableGPRS(true)) {
    halt(F("Failed to turn GPRS on, resetting..."));
  }
  Serial.println(F("Connected to Cellular!"));
  // Wait a little bit to stabilize the connection.
  Watchdog.reset();  
  delay(3000);
  // Now make the MQTT connection.
  int8_t ret = mqtt.connect();
  if (ret != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    halt(F("MQTT connection failed, resetting..."));
  }
  Serial.println(F("MQTT Connected!"));


}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


void loop() {
  Watchdog.reset();  
  // Reset everything if disconnected or too many transmit failures occured in a row.
  if (!fona.TCPconnected() || (txFailures >= MAX_TX_FAILURES)) {
    halt(F("Connection lost, resetting..."));
  }

  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (!isnan(event.temperature)) {
    logTemperature(event.temperature,temperature_feed);
  }
  dht.humidity().getEvent(&event);
  if (!isnan(event.relative_humidity)) {
    logHumidity(event.relative_humidity,humidity_feed);
  }
  
  // Grab battery reading
  uint16_t vbat;
  fona.getBattPercent(&vbat);
  logBatteryPercent(vbat, battery_feed);
  
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (GPS.fix) {
    Serial.print("quality: "); Serial.println((int)GPS.fixquality); 
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", "); 
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print("Location (in degrees, works with Google Maps): ");
    Serial.print(GPS.latitudeDegrees, 4);
    Serial.print(", "); 
    Serial.println(GPS.longitudeDegrees, 4);
    
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    logLocation(GPS.latitudeDegrees,GPS.latitudeDegrees,GPS.altitude,location_feed);
  }

  Serial.println("\n\n\n");
  delay(10000);
}


// Log battery
void logBatteryPercent(uint32_t indicator, Adafruit_MQTT_Publish& publishFeed) {

  // Publish
  Serial.print(F("Publishing battery percentage: "));
  Serial.println(indicator);
  if (!publishFeed.publish(indicator)) {
    Serial.println(F("Publish failed!"));
    txFailures++;
  }
  else {
    Serial.println(F("Publish succeeded!"));
    txFailures = 0;
  }
}

void logTemperature(float temperature,Adafruit_MQTT_Publish& publishFeed){
  Serial.print(F("Publishing Temperature: "));
  Serial.println(temperature);
  if (!publishFeed.publish(temperature)) {
    Serial.println(F("Publish failed!"));
    txFailures++;
  }
  else {
    Serial.println(F("Publish succeeded!"));
    txFailures = 0;
  }
}
void logHumidity(float humidity,Adafruit_MQTT_Publish& publishFeed){
  Serial.print(F("Publishing Humidity: "));
  Serial.println(humidity);
  if (!publishFeed.publish(humidity)) {
    Serial.println(F("Publish failed!"));
    txFailures++;
  }
  else {
    Serial.println(F("Publish succeeded!"));
    txFailures = 0;
  }
}

void halt(const __FlashStringHelper *error) {
  Serial.println(error);
  while (1) {
    digitalWrite(ledPin, LOW);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }
}

void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}


void logLocation(float latitude, float longitude, float altitude, Adafruit_MQTT_Publish& publishFeed) {
  // Initialize a string buffer to hold the data that will be published.
  char sendBuffer[120];
  memset(sendBuffer, 0, sizeof(sendBuffer));
  int index = 0;

  // Start with '0,' to set the feed value.  The value isn't really used so 0 is used as a placeholder.
  sendBuffer[index++] = '0';
  sendBuffer[index++] = ',';

  // Now set latitude, longitude, altitude separated by commas.
  dtostrf(latitude, 2, 6, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  sendBuffer[index++] = ',';
  dtostrf(longitude, 3, 6, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  sendBuffer[index++] = ',';
  dtostrf(altitude, 2, 6, &sendBuffer[index]);

  // Finally publish the string to the feed.
  Serial.print(F("Publishing location: "));
  Serial.println(sendBuffer);
  if (!publishFeed.publish(sendBuffer)) {
    Serial.println(F("Publish failed!"));
    txFailures++;
  }
  else {
    Serial.println(F("Publish succeeded!"));
    txFailures = 0;
  }
}


