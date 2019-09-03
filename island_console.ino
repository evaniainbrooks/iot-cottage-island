#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>
#include <printf.h>

#include <DHT.h>
#include <DHT_U.h>

unsigned long lastPing = 0; // timestamp
unsigned long lastTemperatureSensorRead = 0;
unsigned long lastGasSensorRead = 0;
unsigned long lastCurrentSensorRead = 0; // timestamp
unsigned long connectedSince = 0; // timestamp
unsigned long now = 0; // timestamp
unsigned long nextConnectionAttempt = 0; // timestamp
unsigned long failedConnectionAttempts = 0;
unsigned long sensorIndex = 0;
unsigned long sensorDelayMs;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

#define VERSION_MESSAGE F("Island Console v0.46 03/09/19")

#define AIO_SERVER      "raspberry.home"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "mosquitto"
#define AIO_KEY         "qq211"

#define SERVER_LISTEN_PORT 80
#define MQTT_CONNECT_RETRY_MAX 5
#define ping_INTERVAL_MS 60000
#define CURRENT_SENSOR_INTERVAL_MS 3000
#define GAS_SENSOR_READ_INTERVAL_MS 5000

// LED strip commands
#define On_Off 0xFF02FD
#define Red 0xFF1AE5
#define Green 0xFF9A65
#define Blue 0xFFA25D
#define Purple 0xFF7887
#define White 0xFF22DD
#define Brighter 0xFF3AC5
#define Dimmer 0xFFBA45
#define Flash 0xFFD02F
#define Fade7 0xFFE01F
#define Fade3 0xFF609F
#define Jump3 0xFF20DF
#define Jump7 0xFFA05F
#define Fast 0xFFE817
#define Slow 0xFFC837

// Digital Pin layout
#define OUTSIDE_LIGHT_PIN 5
#define KITCHEN_LIGHT_PIN 6
#define MOTION_SENSOR_PIN 32
#define ISLAND_LIGHT_PIN 33
#define DHT_PIN 36
#define SIDE_DOOR_PIN 37 
#define BUTTON0_PIN 41
#define BUTTON1_PIN 43
#define LED_RED_PIN 44
#define LED_GREEN_PIN 45
#define LED_BLUE_PIN 46
#define LED1_PIN 47 
#define LED0_PIN 48

// Analog pin layout
#define ISLAND_LIGHT_CIRCUIT A1
#define OUTSIDE_LIGHT_CIRCUIT A2
#define KITCHEN_LIGHT_CIRCUIT A3
#define GAS_SENSOR_PIN A4

#define DHT_TYPE DHT11     // DHT 22 (AM2302)
DHT_Unified dht(DHT_PIN, DHT_TYPE);

#define halt(s) { Serial.println(F( s )); while(1);  }

void(* __resetFunc) (void) = 0; //declare reset function @ address 0

void resetFunc(const __FlashStringHelper* msg, unsigned long delayMs) {
  Serial.println(msg);
  Serial.print(F("Resetting in "));
  Serial.print(delayMs / 1000);
  Serial.println(F("s"));
  delay(delayMs);
  __resetFunc();
}

int lastState[80] = {LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW};

EthernetClient client;
EthernetServer server(SERVER_LISTEN_PORT);

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

#define WILL_FEED AIO_USERNAME "/feeds/nodes.island"
//#define LOCK_FEED AIO_USERNAME "/feeds/locks.frontdoor"

Adafruit_MQTT_Publish lastwill = Adafruit_MQTT_Publish(&mqtt, WILL_FEED);

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish front = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/doors.front");
Adafruit_MQTT_Publish side = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/doors.side");
Adafruit_MQTT_Publish motion = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/status.motion");

Adafruit_MQTT_Publish kitchenlight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.kitchenlight");
Adafruit_MQTT_Publish islandlight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.islandlight");
Adafruit_MQTT_Publish outsidelight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.outsidelight");

Adafruit_MQTT_Publish button0 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/island.button0");
Adafruit_MQTT_Publish button1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/island.button1");

Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/island.temperature");
Adafruit_MQTT_Publish humid = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/island.humidity");
Adafruit_MQTT_Publish gas = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/smoke.island");

Adafruit_MQTT_Subscribe ledtoggle = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.led");
Adafruit_MQTT_Subscribe kitchenlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.kitchenlight");
Adafruit_MQTT_Subscribe outsidelight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.outsidelight");
Adafruit_MQTT_Subscribe islandlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.islandlight");

Adafruit_MQTT_Subscribe led0 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/island.led0");
Adafruit_MQTT_Subscribe led1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/island.led1");

Adafruit_MQTT_Subscribe ledred = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/island.ledred");
Adafruit_MQTT_Subscribe ledgreen = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/island.ledgreen");
Adafruit_MQTT_Subscribe ledblue = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/island.ledblue");


float lastTemp;
float lastHumid;
int lastGasSensor;

const int mVperAmp = 100; // use 100 for 20A Module and 66 for 30A Module

double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

void readAcs712(int sensorIn, Adafruit_MQTT_Publish* feed, bool forceUpdate = false) {

 Voltage = getVPP(sensorIn);
 VRMS = (Voltage / 2.0) * 0.707;
 AmpsRMS = (VRMS * 1000) / mVperAmp;
 Serial.print(AmpsRMS);
 Serial.print(F(" Amps RMS on "));
 Serial.println(sensorIn);

 if (AmpsRMS > 0.20) {
   if (lastState[sensorIn] == LOW || forceUpdate) {
     Serial.println(F("Current rising edge detected. Publishing feed status 1"));
     if (feed) { feed->publish("1"); }
   }

   lastState[sensorIn] = HIGH;
 } else {
   if (lastState[sensorIn] == HIGH || forceUpdate) {
     Serial.println(F("Current falling edge detected. Publishing feed status 0"));
     if (feed) { feed->publish("0"); }
   }

   lastState[sensorIn] = LOW;
 }
}

float getVPP(int sensorIn) {
  float result;

  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  int minValue = 1024;          // store min value here

  uint32_t startTime = millis();
  while ((millis() - startTime) < 300) {
    readValue = analogRead(sensorIn);
       
    // see if you have a new maxValue
    if (readValue > maxValue) {
      /* record the maximum sensor value */
      maxValue = readValue;
    }
       
    if (readValue < minValue) {
      /* record the maximum sensor value */
      minValue = readValue;
    }
  }

   // Subtract min from max
   result = ((maxValue - minValue) * 5.0) / 1024.0;

   return result;
 }
 

void setup() {
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  
  delay(2000);

  pinMode(BUTTON0_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);

  // These pins are input pullup, so set the expected default state
  lastState[BUTTON0_PIN] = HIGH;
  lastState[BUTTON1_PIN] = HIGH;
  
  pinMode(SIDE_DOOR_PIN, INPUT_PULLUP);
  lastState[SIDE_DOOR_PIN] = HIGH;
  
  pinMode(MOTION_SENSOR_PIN, INPUT);

  pinMode(OUTSIDE_LIGHT_PIN, OUTPUT);
  pinMode(KITCHEN_LIGHT_PIN, OUTPUT);
  pinMode(ISLAND_LIGHT_PIN, OUTPUT);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

  pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);

  digitalWrite(OUTSIDE_LIGHT_PIN, HIGH);
  digitalWrite(KITCHEN_LIGHT_PIN, HIGH);
  digitalWrite(ISLAND_LIGHT_PIN, HIGH);

  analogWrite(LED_RED_PIN, 0);
  analogWrite(LED_GREEN_PIN, 0);
  analogWrite(LED_BLUE_PIN, 0);

  pinMode(LED0_PIN, LOW);
  pinMode(LED1_PIN, LOW);
  
  Serial.begin(115200);
  printf_begin();
  delay(100);

  Serial.println(VERSION_MESSAGE);

  // Initialise the Client
  Serial.println(F("Joining the network..."));
  Ethernet.begin(mac);
  delay(2000); //give the ethernet a second to initialize
  Serial.println(Ethernet.localIP());
  if (Ethernet.localIP() == IPAddress(0,0,0,0)) {
    resetFunc(F("DHCP resolution failed"), 30000);
  }

  Serial.println("MQTT subscribe");
  mqtt.subscribe(&kitchenlight, &onSubscriptionEvent);
  mqtt.subscribe(&islandlight, &onSubscriptionEvent);
  mqtt.subscribe(&outsidelight, &onSubscriptionEvent);
  mqtt.subscribe(&ledtoggle, &onSubscriptionEvent);

  mqtt.subscribe(&led0, &onSubscriptionEvent);
  mqtt.subscribe(&led1, &onSubscriptionEvent);
  
  mqtt.subscribe(&ledred, &onLedStripSubscriptionEvent);
  mqtt.subscribe(&ledblue, &onLedStripSubscriptionEvent);
  mqtt.subscribe(&ledgreen, &onLedStripSubscriptionEvent);

  mqtt.will(WILL_FEED, "0");

  delay(1000);
  
  readAcs712(KITCHEN_LIGHT_CIRCUIT, &kitchenlight_pub, true);
  readAcs712(ISLAND_LIGHT_CIRCUIT, &islandlight_pub, true);
  readAcs712(OUTSIDE_LIGHT_CIRCUIT, &outsidelight_pub, true);

  initSensor();
  lastGasSensor = analogRead(GAS_SENSOR_PIN);
  lastGasSensorRead = millis();
  Serial.println(F("Finished init"));
}

void initSensor() {
  // Initialize device.
  dht.begin();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
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
  Serial.println(F("------------------------------------"));
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.print  ("Min Delay:    "); Serial.print(sensor.min_delay / 1000); Serial.println("ms");

  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  //sensorDelayMs = sensor.min_delay / 1000;
  sensorDelayMs = 30000;
}


void onLedStripSubscriptionEvent(Adafruit_MQTT_Subscribe* subscription) {
  Serial.println(F("Incoming message"));
  int val = atoi((char*)subscription->lastread);
  
  if (subscription == &ledred) {

    analogWrite(LED_RED_PIN, val);
    lastState[LED_RED_PIN] = val;
    
  } else if (subscription == &ledgreen) {
    
    analogWrite(LED_GREEN_PIN, val);
    lastState[LED_GREEN_PIN] = val;
    
  } else if (subscription == &ledblue) {
    
    analogWrite(LED_BLUE_PIN, val);
    lastState[LED_BLUE_PIN] = val;
  }
}

void onSubscriptionEvent(Adafruit_MQTT_Subscribe* subscription) {
  
  Serial.println(F("Incoming message"));
  char* command = (char*)subscription->lastread;
  command[0] = tolower(command[0]);
  if (subscription == &outsidelight) {

    Serial.println(F("msg: OUTSIDE LIGHT"));
    handleMqttToggleCommand(command, OUTSIDE_LIGHT_PIN, OUTSIDE_LIGHT_CIRCUIT);

  } else if (subscription == &kitchenlight) {

    Serial.println(F("msg: KITCHEN LIGHT"));
    handleMqttToggleCommand(command, KITCHEN_LIGHT_PIN, KITCHEN_LIGHT_CIRCUIT);

  } else if (subscription == &islandlight) {

    Serial.println(F("msg: ISLAND LIGHT"));
    handleMqttToggleCommand(command, ISLAND_LIGHT_PIN, ISLAND_LIGHT_CIRCUIT);

  } else if (subscription == &led0) {

    Serial.println(F("msg: LED0"));
    handleMqttToggleLedCommand(command, LED0_PIN);

  } else if (subscription == &led1) {

    Serial.println(F("msg: LED1"));
    handleMqttToggleLedCommand(command, LED1_PIN);
  }
}


void readGasSensor(bool force = false) {
  if (force || now - lastGasSensorRead > GAS_SENSOR_READ_INTERVAL_MS) {
    uint32_t sensor = analogRead(GAS_SENSOR_PIN);
    bool sensorChanged = abs(lastGasSensor - sensor) > 50;
    bool sensorAboveThreshold = sensor > 400;

    lastGasSensorRead = now;
    lastGasSensor = sensor;

    if (force || sensorChanged || sensorAboveThreshold) {
      gas.publish(sensor);
    }
  }
}

void loop() {
  now = millis();
  Ethernet.maintain();
  connectMqtt();

  mqtt.process(50);
  //mqttRadioBridge.process(50);


  detectEdge(SIDE_DOOR_PIN, &side);
  detectEdge(MOTION_SENSOR_PIN, &motion);

  detectEdge(BUTTON0_PIN, &button0);

  detectEdge(BUTTON1_PIN, &button1);
  
  handleHttpClientRequest();
  readCurrentSensors();

  ping();
  
    // Get temperature event and print its value.
  if (now - lastTemperatureSensorRead > sensorDelayMs) {
    Serial.println(F("sensorRead"));
    lastTemperatureSensorRead = now;
    readSensor();
  }

  readGasSensor();

  delay(10);
}

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

  lastTemp = event.temperature;
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

  lastHumid = event.relative_humidity;
  humid.publish(event.relative_humidity);
}

void readCurrentSensors() {
  if (now - lastCurrentSensorRead > CURRENT_SENSOR_INTERVAL_MS) {
    lastCurrentSensorRead = now;

    if (sensorIndex == 0) {
      Serial.println(F("Read kitchen light current"));
      readAcs712(KITCHEN_LIGHT_CIRCUIT, &kitchenlight_pub);
    }
    if (sensorIndex == 1) {
      Serial.println(F("Read island light current"));
      readAcs712(ISLAND_LIGHT_CIRCUIT, &islandlight_pub);
    }
    if (sensorIndex == 2) {
      Serial.println(F("Read outside light current"));
      readAcs712(OUTSIDE_LIGHT_CIRCUIT, &outsidelight_pub);
    }
    
    sensorIndex = (sensorIndex + 1) % 3;
  }
}

void handleMqttToggleLedCommand(const char* command, int outputPin) {
    if (strcmp((char *)command, "1") == 0) {
      Serial.print(F("Turning on "));
      Serial.println(outputPin);
      lastState[outputPin] = HIGH;
      digitalWrite(outputPin, lastState[outputPin]);
    } else if (strcmp((char *)command, "0") == 0) {
      Serial.print(F("Turning off "));
      Serial.println(outputPin);
      lastState[outputPin] = LOW;
      digitalWrite(outputPin, lastState[outputPin]);
    } else {
      Serial.print(F("Unknown command"));
      Serial.println((char*)command);
    }
}

void handleMqttToggleCommand(const char* command, int outputPin, int acsPin) {

  if (-1 == acsPin) {
    
    // For lights with no attached ACS sensr
    if (strcmp((char *)command, "1") == 0 && lastState[outputPin] == HIGH) {
      Serial.println(F("ACS-less relay is OFF (turn on)"));
      lastState[outputPin] = LOW;
      Serial.print(F("Setting state to "));
      Serial.print(lastState[outputPin]);
      digitalWrite(outputPin, lastState[outputPin]);

    } else if (strcmp((char *)command, "0") == 0 && lastState[outputPin] == LOW) {
      Serial.println(F("ACS-less relay is ON (turn off)"));
      lastState[outputPin] = HIGH;
      Serial.print(F("Setting state to "));
      Serial.print(lastState[outputPin]);
      digitalWrite(outputPin, lastState[outputPin]);
    }
  } else {

    // For lights with attached ACS sensor
    if (strcmp((char *)command, "1") == 0 && lastState[acsPin] == LOW) {
      Serial.println(F("ACS relay is OFF (turn on)"));
      lastState[outputPin] = lastState[outputPin] == HIGH ? LOW : HIGH;
      lastState[acsPin] = HIGH; // Optimistic toggling of ACS pin state. This avoids an extra rising edge message being published
      Serial.print(F("Setting state to "));
      Serial.print(lastState[outputPin]);
      digitalWrite(outputPin, lastState[outputPin]);
      
    } else if (strcmp((char *)command, "0") == 0 && lastState[acsPin] == HIGH) {
      Serial.println(F("ACS relay is ON (turn off)"));
      lastState[outputPin] = lastState[outputPin] == HIGH ? LOW : HIGH;
      lastState[acsPin] = lastState[acsPin] == HIGH ? LOW : HIGH;  // Optimistic toggling of ACS pin state. This avoids an extra falling edge message being published
      Serial.print(F("Setting state to "));
      Serial.print(lastState[outputPin]);
      digitalWrite(outputPin, lastState[outputPin]);
    } else if (strcmp((char*)command, "2") == 0) {
      Serial.println(F("ACS relay toggle"));
      lastState[outputPin] = lastState[outputPin] == HIGH ? LOW : HIGH;
      lastState[acsPin] = lastState[acsPin] == HIGH ? LOW : HIGH; // Optimistic toggling of ACS pin state
      digitalWrite(outputPin, lastState[outputPin]);
    }
  }
}

void detectEdge(int pin, Adafruit_MQTT_Publish* feed) {
  int state = digitalRead(pin);
  if (state != lastState[pin]) {
    Serial.print(F("Publishing state change on pin "));
    Serial.print(pin);
    if (state == HIGH) {
      Serial.println(F(", 1"));
      feed->publish("1");
    } else {
      Serial.println(F(", 0"));
      feed->publish("0");
    }
  }

  lastState[pin] = state;
}

void onPing(bool result) {
  readGasSensor(true);
  lastwill.publish(now);
}

void ping() {

  if (!mqtt.connected()) {
    return;
  }

  if (now - lastPing > ping_INTERVAL_MS) {
    Serial.println(F("Ping"));
    lastPing = now;
    mqtt.pingAsync(onPing);
  }
}

void connectMqtt() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  if (nextConnectionAttempt < now) {
    Serial.print(F("Connecting to MQTT... "));

    int delaySecs = (2 << failedConnectionAttempts); // Delay for 2, 4, 8 .. seconds
    if (ret = mqtt.connect() != 0) {
      Serial.print(F("Failed: "));
      Serial.println(mqtt.connectErrorString(ret));
      //mqtt.disconnect();

      nextConnectionAttempt = now + delaySecs * 1000;
      ++failedConnectionAttempts;
    }
  
    if (0 == ret) {
      connectedSince = millis();
      failedConnectionAttempts = 0;
      Serial.println(F("Connected!"));
    } else if (failedConnectionAttempts > MQTT_CONNECT_RETRY_MAX) {
      connectedSince = 0;
      resetFunc(F("Max retries exhausted!"), 2000); // Reset and try again
    } else {
      Serial.print(F("Retrying in "));

      Serial.print(delaySecs);
      Serial.println(F("s"));
    }
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

          client.print(F("<h1>"));
          client.print(VERSION_MESSAGE);
          client.println(F("</h1>"));
          client.print(F("<br />Led0 is "));
          client.println(lastState[LED0_PIN]);
          client.print(F("<br />Led1 is "));
          client.println(lastState[LED1_PIN]);
          
          client.print(F("<br />Outside light is "));
          client.println(lastState[OUTSIDE_LIGHT_PIN]);
          client.print(F("<br />Kitchen light is "));
          client.println(lastState[KITCHEN_LIGHT_PIN]);
          client.print(F("<br />Island light is "));
          client.println(lastState[ISLAND_LIGHT_PIN]);
          client.print(F("<br />Front door is "));
          client.println(lastState[SIDE_DOOR_PIN]);
          client.print(F("<br />Temperature "));
          client.print(lastTemp);
          client.print(F("<br />Humidity "));
          client.print(lastHumid);
          client.print(F("<br />Smoke "));
          client.print(lastGasSensor);
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

