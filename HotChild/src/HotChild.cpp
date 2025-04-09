/* 
 * Project Capstone Project HotChild
 * Author: Ben Njus
 * Date: 04/18/2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_GPS.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Credentials.h"
#include "Adafruit_BME280.h"
#include "MP3_CNM.h"
#include "JsonParserGeneratorRK.h"

TCPClient TheClient;
Adafruit_GPS GPS(&Wire);
Adafruit_BME280 bme;
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

// Define Constants
const int TIMEZONE = -6;
const unsigned int UPDATE = 30000;
float TempHotChild;
float LatandLon;

float lat, lon, alt;
int sat;
unsigned int lastGPS;
bool status;

//SYSTEM_MODE(SEMI_AUTOMATIC);

Adafruit_MQTT_Subscribe subFeed = Adafruit_MQTT_Subscribe(&mqtt,AIO_USERNAME "/feeds/feed1");
Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME "/feeds/TempHotChild");
Adafruit_MQTT_Publish pubFeed1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/LatandLon");

void getGPS(float *latitude, float *longitude, float *altitude, int *satellites);
unsigned int last, lastTime;
unsigned int currentTime;
unsigned int lastSecond;
float subValue,pubValue;
const int hexAddress = 0x76; // BME280 address

void MQTT_connect();
bool MQTT_ping();

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,5000);
  //WiFi.on();
  //WiFi.clearCredentials(); //prevent from connecting to DDCIOT
  //WiFi.setCredentials("IoTNetwork");
  //WiFi.connect(); //Connect to internet, but not Particle Cloud
 // while(WiFi.connecting()) {
  //Serial.printf(".");
 // }
  //Initialize GPS
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);
  status = bme.begin(hexAddress);
  if (status == false) {
  Serial.printf("BME280 at address 0x%02X failed to start", hexAddress);
  mqtt.subscribe(&subFeed);
  }
 }


void loop() {
MQTT_connect();
MQTT_ping();
// Get data from GSP unit (best if you do this continuously)
GPS.read();
if (GPS.newNMEAreceived()) {
  if (!GPS.parse(GPS.lastNMEA())) {
    return;
  }   
}

if (millis() - lastGPS > UPDATE) {
  lastGPS = millis(); // reset the timer
  getGPS(&lat,&lon,&alt,&sat);
  Serial.printf("\n=================================================================\n");
  Serial.printf("Lat: %0.6f, Lon: %0.6f, Alt: %0.6f, Satellites: %i\n",lat, lon, alt, sat);
  Serial.printf("=================================================================\n\n");
}
TempHotChild = bme.readTemperature();
if (TempHotChild > 30 ) {
  if(mqtt.Update()) { //if mqtt object (Adafruit.io) is available to receive data
    Serial.printf("Publishing %0.2f to Adafruit.io feed FeedNameB \n",TempHotChild);
    pubFeed.publish(TempHotChild);
    Serial.printf("Publishing %0.2f to Adafruit.io feed FeedNameB \n",LatandLon);
    pubFeed1.publish(lat,lon);
   // Receive data from a subscription to an MQTT feed
   // Adafruit_MQTT_Subscribe *subscription;
   // while ((subscription = mqtt.readSubscription(100))) { //wait a moment for new feed data
   //   if (subscription == &subFeed) { // assign new data to appropriate variable
   //   value2 = atof((char *)subFeed.lastread); //value2 = data from MQTT subscription
   //   Serial.printf("Received %0.2f from Adafruit.io feed FeedNameB \n",value2);
   //   }
    }
  }

}


void getGPS(float *latitude, float *longitude, float *altitude, int *satellites){
  int theHour;

  theHour = GPS.hour + TIMEZONE;
  if(theHour < 0) {
    theHour = theHour + 24;
  }
    
  Serial.printf("Time: %02i:%02i:%02i:%03i\n",theHour, GPS.minute, GPS.seconds, GPS.milliseconds);
  Serial.printf("Dates: %02i-%02i-20%02i\n", GPS.month, GPS.day, GPS.year);
  Serial.printf("Fix: %i, Quality: %i",(int)GPS.fix,(int)GPS.fixquality);
    if (GPS.fix) {
      *latitude = GPS.latitudeDegrees;
      *longitude = GPS.longitudeDegrees; 
      *altitude = GPS.altitude;
      *satellites = (int)GPS.satellites;
    }
}

void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}