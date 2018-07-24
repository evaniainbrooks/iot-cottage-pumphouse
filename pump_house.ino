
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>
#include <Adafruit_Sensor.h>

#include <DHT.h>
#include <DHT_U.h>

//
// Plug layout
//
// NO [ x3 | x2 ] NC
//    [ x4 | x1 ]
//
// Relay control colors (OUTPUT; HIGH is on)
//
// x1 B
// x2 G
// x3 Y
// x4 O
//
// x1 Attached to 20 AMP ACS712 (ANALOG IN)
//
// VCC B
// GND P
// DATA G
/************************* Ethernet Client Setup *****************************/
byte mac[] = {0xFE, 0xED, 0xDD, 0xAD, 0xBD, 0xEF};

//Uncomment the following, and set to a valid ip if you don't have dhcp available.
IPAddress iotIP (192, 168, 2, 102);

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "abc"
#define AIO_KEY         "xyz"

#define ETHERNET_SHIELD_RESET_PIN 3

#define RELAY_PIN_0 2 // x4
#define RELAY_PIN_1 3 // x3
#define RELAY_PIN_2 5 // x2
#define RELAY_PIN_3 6 // x1
#define DHT_PIN 7

// Uncomment the type of sensor in use:
#define DHT_TYPE DHT22     // DHT 22 (AM2302)

#define VERSION_MESSAGE F("Pump House Console v0.12 24/07/18")

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHT_PIN, DHT_TYPE);

uint32_t delayMS;

//Set up the ethernet client
EthernetClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

void(* __resetFunc) (void) = 0; //declare reset function @ address 0

void resetFunc(const __FlashStringHelper* msg) {
  Serial.println(msg);
  Serial.println(F("Resetting in 2 seconds"));
  delay(2000);
  __resetFunc();
}


/****************************** Feeds ***************************************/

#define WILL_FEED AIO_USERNAME "/feeds/nodes.pumphouse"
Adafruit_MQTT_Publish lastwill = Adafruit_MQTT_Publish(&mqtt, WILL_FEED);

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humid = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish statuswellpump = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/status.wellpump");

Adafruit_MQTT_Subscribe togglewellpump = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.wellpump");
Adafruit_MQTT_Subscribe toggleheating = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.heatingcable");

// Possibly as
//  #define TOGGLE_TV "1"
//  #define TOGGLE_SOUNDBAR "2"
//  Adafruit_MQTT_Subscribe devicetoggle = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/devicetoggle");
//

/*************************** Sketch Code ************************************/

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

//const char lastError* = null;
//unsigned long lastErrorTime = 0;

void initSensor() {
  // Initialize device.
  dht.begin();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}

void setup() {

  Serial.begin(115200);

  Serial.println(VERSION_MESSAGE);

  pinMode(RELAY_PIN_0, OUTPUT);
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT);

  digitalWrite(RELAY_PIN_0, HIGH);
  digitalWrite(RELAY_PIN_1, HIGH);
  digitalWrite(RELAY_PIN_0, LOW);
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, HIGH);
  digitalWrite(RELAY_PIN_3, HIGH);

  // disable SD card
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  // Initialise the Client
  Serial.println(F("Joining the network..."));
  Ethernet.begin(mac);

  delay(1000); //give the ethernet a second to initialize

  Serial.print(F("Server is at "));
  Serial.println(Ethernet.localIP());
  if (Ethernet.localIP() == IPAddress(0,0,0,0)) {
    resetFunc(F("Failed Ethernet.begin"));
  }

  delay(250);

  //MQTT_connect();
  Serial.println(F("MQTT subscribe"));

  //mqtt.subscribe(&lamptoggle);
  mqtt.subscribe(&togglewellpump);
  mqtt.will(WILL_FEED, "0");

  server.begin();

  //Serial.println("Enable IR");
  //irrecv.enableIRIn();
  //irrecv.blink13(true);

  initSensor();
  /*
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
    delay(500);                       // wait for half a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
    delay(500);                       // wait for half a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off
    delay(500);

    Serial.println(F("Finished init"));*/
}


unsigned long lastSensorRead = 0;

void readSensor() {

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F(" *C"));
  }

  temp.publish(event.temperature);

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }

  humid.publish(event.relative_humidity);
}

/*
Measuring Current Using ACS712
*/
int sensorIn = A1;
int mVperAmp = 100; // use 100 for 20A Module and 66 for 30A Module
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

int currentDetected = LOW;

void readAcs712() {

Voltage = getVPP();
 VRMS = (Voltage/2.0) *0.707;
 AmpsRMS = (VRMS * 1000)/mVperAmp;
 //Serial.print(AmpsRMS);
 //Serial.println(F(" Amps RMS"));

 if (AmpsRMS > 0.1) {
   if (currentDetected == LOW) {
     Serial.println(F("Publishing status.wellpump 1"));
     statuswellpump.publish("1");
   }

   currentDetected = HIGH;
 } else {
   if (currentDetected == HIGH) {
     Serial.println(F("Publishing status.wellpump 0"));
     statuswellpump.publish("0");
   }

   currentDetected = LOW;
 }
}

float getVPP()
{
  float result;

  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  int minValue = 1024;          // store min value here

   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(sensorIn);
       // see if you have a new maxValue
       if (readValue > maxValue)
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue)
       {
           /*record the maximum sensor value*/
           minValue = readValue;
       }
   }

   // Subtract min from max
   result = ((maxValue - minValue) * 5.0)/1024.0;

   return result;
 }


unsigned long now;

void loop() {
  now = millis();

  Ethernet.maintain();
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    //Serial.println("Incoming message");

    Serial.print(F("Incoming MQTT message: "));
    if (subscription == &togglewellpump) {

      Serial.print(F("toggle well pump"));
      Serial.println((char *)subscription->lastread);

      if (strcmp((char *)subscription->lastread, "1") == 0) {
        digitalWrite(RELAY_PIN_3, HIGH);
      }
      if (strcmp((char *)subscription->lastread, "0") == 0) {
        digitalWrite(RELAY_PIN_3, LOW);
      }
    } else if (subscription == &toggleheating) {
      Serial.print(F("toggle heating cable"));
      Serial.println((char *)subscription->lastread);

      if (strcmp((char *)subscription->lastread, "1") == 0) {
        digitalWrite(RELAY_PIN_2, HIGH);
      }
      if (strcmp((char *)subscription->lastread, "0") == 0) {
        digitalWrite(RELAY_PIN_2, LOW);
      }
    }
  }

  // Get temperature event and print its value.
  if (lastSensorRead + 30000 < now) {
    Serial.println(F("sensorRead"));
    lastSensorRead = now;
    readSensor();

  }

  //IR_decode();
  MQTT_ping();

  handleHttpClientRequest();

  readAcs712();

  delay(200);
  //Serial.print(F("Loop "));
  //Serial.println(now);
}

unsigned long lastPing = 0;

void MQTT_ping() {

  if (lastPing + 60000 < now) {
    Serial.println(F("Ping"));
    lastPing = now;
    if (!mqtt.ping()) {
      Serial.println(F("Failed to ping"));
      mqtt.disconnect();
    } else {
      lastwill.publish(String(now, DEC).c_str());
    }
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.

unsigned long connectedSince = 0;

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print(F("Connecting to MQTT... "));

  int attempts = 0;
  while (++attempts < 10 && (ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.print(F("Retrying MQTT connection in "));

    int delaySecs = (2 << attempts); // Delay for 2, 4, 8 .. seconds
    Serial.print(delaySecs);
    Serial.println(F(" seconds"));
    mqtt.disconnect();
    delay(delaySecs * 1000);
  }

  if (0 == ret) {
    connectedSince = millis();
    Serial.println(F("MQTT Connected!"));
  } else {
    connectedSince = 0;
    resetFunc(F("Failed connection!")); // Reset and try again
  }
}


void handleHttpClientRequest() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println(F("New http client"));
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: text/html"));
          client.println(F("Connection: close"));  // the connection will be closed after completion of the response
          client.println(F("Refresh: 10"));  // refresh the page automatically every 5 sec
          client.println();
          client.println(F("<!DOCTYPE HTML>"));
          client.println(F("<html>"));
          // output the value of each analog input pin

          int pump = digitalRead(RELAY_PIN_3);
          int heater = digitalRead(RELAY_PIN_2);
          client.print(F("<h1>"));
          client.print(VERSION_MESSAGE);
          client.print(F("</h1>"));

          client.print(F("<br />Pump is "));
          client.println(!pump);
          client.print(F("<br />Heating cable is "));
          client.println(!heater);
          client.print(F("<br />Last amps reading "));
          client.print(AmpsRMS);

          client.print(F("<br />Last ping "));
          client.print(lastPing);
          client.print(F("<br />Uptime "));
          client.print(now);
          client.print(F("<br />Connected since "));
          client.print(connectedSince);

          client.println(F("<br />"));

          client.println(F("</html>"));
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println(F("Http client disconnected"));
  }
}

