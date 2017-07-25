// Geo Fencing project with Adafruit IO
// Author: Marco Schwartz
//
// Inspired by the code from Tony DiCola
//
// Released under a MIT license:
// https://opensource.org/licenses/MIT

// Libraries
#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"

// Alarm pins
const int ledPin = 6;

// FONA pins configuration
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

// FONA GPRS configuration
#define FONA_APN             ""  // APN used by cell data service (leave blank if unused)

// Adafruit IO configuration
#define AIO_SERVER           "io.adafruit.com"  // Adafruit IO server name.
#define AIO_SERVERPORT       1883  // Adafruit IO port.
#define AIO_USERNAME         "alexvargasbenamburg"  // Adafruit IO username (see http://accounts.adafruit.com/).
#define AIO_KEY              "1d33a5c00560487eac6b5483a4fb3038"  // Adafruit IO key (see settings page at: https://io.adafruit.com/settings).

// Feeds
#define MAX_TX_FAILURES      3  // Maximum number of publish failures in a row before resetting the whole sketch.

// FONA instance & configuration
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);     // FONA software serial connection.
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);                 // FONA library connection.

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER,AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

uint8_t txFailures = 0;                                       // Count of how many publish failures have occured in a row.

const char BATTERY_FEED[] = AIO_USERNAME "/feeds/battery";
Adafruit_MQTT_Publish battery_feed = Adafruit_MQTT_Publish(&mqtt, BATTERY_FEED);

void setup() {
  
  // Initialize serial output.
  Serial.begin(115200);
  Serial.println(F("Geofencing with Adafruit IO & FONA808"));

  // Set alarm components
  pinMode(ledPin, OUTPUT);

  // Initialize the FONA module
  Serial.println(F("Initializing FONA....(may take 10 seconds)"));
  fonaSS.begin(4800);
  if (!fona.begin(fonaSS)) {
    halt(F("Couldn't find FONA"));
  }
  fonaSS.println("AT+CMEE=2");
  Serial.println(F("FONA is OK"));

  // Use the watchdog to simplify retry logic and make things more robust.
  // Enable this after FONA is intialized because FONA init takes about 8-9 seconds.
  Watchdog.enable(8000);
  Watchdog.reset();

  // Wait for FONA to connect to cell network (up to 8 seconds, then watchdog reset).
  Serial.println(F("Checking for network..."));
  while (fona.getNetworkStatus() != 1) {
   delay(500);
  }

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

void loop() {
  
  // Watchdog reset at start of loop--make sure everything below takes less than 8 seconds in normal operation!
  Watchdog.reset();

  // Reset everything if disconnected or too many transmit failures occured in a row.
  if (!fona.TCPconnected() || (txFailures >= MAX_TX_FAILURES)) {
    halt(F("Connection lost, resetting..."));
  }

  // Grab battery reading
  uint16_t vbat;
  fona.getBattPercent(&vbat);

  logBatteryPercent(vbat, battery_feed);
  
  // Wait 5 seconds
  delay(5000);

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


// Halt function called when an error occurs.  Will print an error and stop execution while
// doing a fast blink of the LED.  If the watchdog is enabled it will reset after 8 seconds.
void halt(const __FlashStringHelper *error) {
  Serial.println(error);
  while (1) {
    digitalWrite(ledPin, LOW);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
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

void printFloat(float value, int places) {
  // this is used to cast digits 
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  int i;
  float tempfloat = value;

    // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0)
    d *= -1.0;
  // divide by ten for each decimal place
  for (i = 0; i < places; i++)
    d/= 10.0;    
  // this small addition, combined with truncation will round our values properly 
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0)
    tempfloat *= -1.0;
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }


  // write out the negative if needed
  if (value < 0)
    Serial.print('-');

  if (tenscount == 0)
    Serial.print(0, DEC);

  for (i=0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    Serial.print(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }

  // if no places after decimal, stop now and return
  if (places <= 0)
    return;

  // otherwise, write the point and continue on
  Serial.print('.');  

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    Serial.print(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }
}


