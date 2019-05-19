#include <IRremote.h>
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>

unsigned long lastPing = 0; // timestamp
unsigned long lastCurrentSensorRead = 0; // timestamp
unsigned long connectedSince = 0; // timestamp
unsigned long now = 0; // timestamp
unsigned long nextConnectionAttempt = 0; // timestamp
unsigned long failedConnectionAttempts = 0;
unsigned long sensorIndex = 0;

/************************* Ethernet Client Setup *****************************/
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

//Uncomment the following, and set to a valid ip if you don't have dhcp available.
//IPAddress iotIP (192, 168, 0, 42);
//Uncomment the following, and set to your preference if you don't have automatic dns.
//IPAddress dnsIP (8, 8, 8, 8);
//If you uncommented either of the above lines, make sure to change "Ethernet.begin(mac)" to "Ethernet.begin(mac, iotIP)" or "Ethernet.begin(mac, iotIP, dnsIP)"

#define VERSION_MESSAGE F("Island Console v0.20 25/08/18")

#define AIO_SERVER      "192.168.2.20"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "mosquitto"
#define AIO_KEY         "qq211"

#define SERVER_LISTEN_PORT 80
#define MQTT_CONNECT_RETRY_MAX 5
#define MQTT_PING_INTERVAL_MS 60000
#define CURRENT_SENSOR_INTERVAL_MS 3000

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
#define FRONT_DOOR_PIN 2
#define MOTION_SENSOR_PIN 32
#define OUTSIDE_LIGHT_PIN 5
#define KITCHEN_LIGHT_PIN 6
#define ISLAND_LIGHT_PIN 33
#define SIDE_DOOR_PIN 8
#define IR_PIN 9

// Analog pin layout
#define ISLAND_LIGHT_CIRCUIT A1
#define OUTSIDE_LIGHT_CIRCUIT A2
#define KITCHEN_LIGHT_CIRCUIT A3

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

// Controlling the LED strip
IRsend irsend;

EthernetClient client;
EthernetServer server(SERVER_LISTEN_PORT);

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

#define WILL_FEED AIO_USERNAME "/feeds/nodes.island"

Adafruit_MQTT_Publish lastwill = Adafruit_MQTT_Publish(&mqtt, WILL_FEED);

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish front = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/doors.front");
Adafruit_MQTT_Publish side = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/doors.side");
Adafruit_MQTT_Publish motion = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/status.motion");

Adafruit_MQTT_Publish kitchenlight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.kitchenlight");
Adafruit_MQTT_Publish islandlight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.islandlight");
Adafruit_MQTT_Publish outsidelight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.outsidelight");

Adafruit_MQTT_Subscribe ledtoggle = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.led");
Adafruit_MQTT_Subscribe kitchenlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.kitchenlight");
Adafruit_MQTT_Subscribe outsidelight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.outsidelight");
Adafruit_MQTT_Subscribe islandlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.islandlight");

void setup() {

  // Disable SD card
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  
  delay(2000);

  pinMode(SIDE_DOOR_PIN, INPUT_PULLUP);
  pinMode(FRONT_DOOR_PIN, INPUT_PULLUP);
  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(IR_PIN, OUTPUT);

  pinMode(OUTSIDE_LIGHT_PIN, OUTPUT);
  pinMode(KITCHEN_LIGHT_PIN, OUTPUT);
  pinMode(ISLAND_LIGHT_PIN, OUTPUT);

  digitalWrite(OUTSIDE_LIGHT_PIN, HIGH);
  digitalWrite(KITCHEN_LIGHT_PIN, HIGH);
  digitalWrite(ISLAND_LIGHT_PIN, HIGH);

  Serial.begin(115200);

  Serial.println(VERSION_MESSAGE);

  // Initialise the Client
  Serial.println(F("Joining the network..."));
  Ethernet.begin(mac);
  delay(2000); //give the ethernet a second to initialize
  Serial.println(Ethernet.localIP());
  if (Ethernet.localIP() == IPAddress(0,0,0,0)) {
    resetFunc(F("DHCP resolution failed"), 30000);
  }

  //MQTT_connect();
  Serial.println("MQTT subscribe");
  //mqtt.subscribe(&tvtoggle);
  mqtt.subscribe(&kitchenlight, &onSubscriptionEvent);
  mqtt.subscribe(&islandlight, &onSubscriptionEvent);
  mqtt.subscribe(&outsidelight, &onSubscriptionEvent);
  mqtt.subscribe(&ledtoggle, &onSubscriptionEvent);

  mqtt.will(WILL_FEED, "0");

  Serial.println("Enable IR");
}

void onSubscriptionEvent(Adafruit_MQTT_Subscribe* subscription) {
  
  Serial.println(F("Incoming message"));
    //irrecv.enableIRIn();
    //irrecv.resume();

   // if (subscription == &tvtoggle) {
     // Serial.println("msg: TOGGLE TV");
     // irsend.sendRC5(0xc, 4);
    //} else
    char* command = (char*)subscription->lastread;
    command[0] = tolower(command[0]);
    if (subscription == &ledtoggle) {
      
      Serial.print(F("msg: TOGGLE LED "));
      Serial.println(command);

      if (strstr(command, "on") != NULL || strstr(command, "off") != NULL) {
        irsend.sendNEC(On_Off, 32);
        delay(40);
      }

      if (strstr(command, "blue") != NULL) {
        irsend.sendNEC(Blue, 32);
        delay(40);
      } else if (strstr(command, "red") != NULL) {
        irsend.sendNEC(Red, 32);
        delay(40);
      } else if (strstr(command, "green") != NULL) {
        irsend.sendNEC(Green, 32);
        delay(40);
      } else if (strstr(command, "white") != NULL) {
        irsend.sendNEC(White, 32);
        delay(40);
      } else if (strstr(command, "blink") != NULL || strstr(command, "flash") != NULL) {
        irsend.sendNEC(Flash, 32);
        delay(40);
      } else if (strstr(command, "purple") != NULL) {
        irsend.sendNEC(Purple, 32);
        delay(40);
      }

      if (strstr(command, "brighter") != NULL) {
        irsend.sendNEC(Brighter, 32);
        delay(40);
      } else if (strstr(command, "dimmer") != NULL) {
        irsend.sendNEC(Dimmer, 32);
        delay(40);
      }

      if (strstr(command, "fade") != NULL) {
        irsend.sendNEC(Fade7, 32);
        delay(40);
      } else if (strstr(command, "strobe") != NULL) {
        irsend.sendNEC(Jump7, 32);
        delay(40);
      }

      if (strstr(command, "slow") != NULL) {
        irsend.sendNEC(Slow, 32);
        delay(40);
      } else if (strstr(command, "fast") != NULL) {
        irsend.sendNEC(Fast, 32);
        delay(40);
      }

      if (strstr(command, "brightest")  != NULL) {
        for (int i = 0; i < 13; ++i) {
          irsend.sendNEC(Brighter, 32);
          delay(40);
        }
      }

      if (strstr(command, "dimmest") != NULL) {
        for (int i = 0; i < 13; ++i) {
          irsend.sendNEC(Dimmer, 32);
          delay(40);
        }
      }

      //irsend.sendNEC(0xFF02FD, 32);
    } else if (subscription == &outsidelight) {

      Serial.println(F("msg: OUTSIDE LIGHT"));
      handleMqttToggleCommand(command, OUTSIDE_LIGHT_PIN, OUTSIDE_LIGHT_CIRCUIT);

    } else if (subscription == &kitchenlight) {

      Serial.println(F("msg: KITCHEN LIGHT"));
      handleMqttToggleCommand(command, KITCHEN_LIGHT_PIN, KITCHEN_LIGHT_CIRCUIT);

    } else if (subscription == &islandlight) {

      Serial.println(F("msg: ISLAND LIGHT"));
      handleMqttToggleCommand(command, ISLAND_LIGHT_PIN, ISLAND_LIGHT_CIRCUIT);

    } 
}

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  now = millis();
  Ethernet.maintain();
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  mqtt.process(50);

  detectEdge(FRONT_DOOR_PIN, &front);
  detectEdge(SIDE_DOOR_PIN, &side);
  detectEdge(MOTION_SENSOR_PIN, &motion);
  
  handleHttpClientRequest();

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

  //IR_decode();
  MQTT_ping();

  delay(50);
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
      Serial.print(F("Setting state to "));
      Serial.print(lastState[outputPin]);
      digitalWrite(outputPin, lastState[outputPin]);
      
    } else if (strcmp((char *)command, "0") == 0 && lastState[acsPin] == HIGH) {
      Serial.println(F("ACS relay is ON (turn off)"));
      lastState[outputPin] = lastState[outputPin] == HIGH ? LOW : HIGH;
      Serial.print(F("Setting state to "));
      Serial.print(lastState[outputPin]);
      digitalWrite(outputPin, lastState[outputPin]);
    } else if (strcmp((char*)command, "2") == 0) {
      Serial.println(F("ACS relay toggle"));
      lastState[outputPin] = lastState[outputPin] == HIGH ? LOW : HIGH;
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
  lastwill.publish(now);
}

void MQTT_ping() {

  if (!mqtt.connected()) {
    return;
  }

  if (now - lastPing > MQTT_PING_INTERVAL_MS) {
    Serial.println(F("Ping"));
    lastPing = now;
    mqtt.pingAsync(onPing);
  }
}

void MQTT_connect() {
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
          client.print(F("<br />Outside light is "));
          client.println(lastState[OUTSIDE_LIGHT_PIN]);
          client.print(F("<br />Kitchen light is "));
          client.println(lastState[KITCHEN_LIGHT_PIN]);
          client.print(F("<br />Island light is "));
          client.println(lastState[ISLAND_LIGHT_PIN]);
          client.print(F("<br />Front door is "));
          client.println(lastState[FRONT_DOOR_PIN]);
          client.print(F("<br />Side door is "));
          client.println(lastState[SIDE_DOOR_PIN]);
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

const int mVperAmp = 100; // use 100 for 20A Module and 66 for 30A Module

double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

void readAcs712(int sensorIn, Adafruit_MQTT_Publish* feed) {

 Voltage = getVPP(sensorIn);
 VRMS = (Voltage / 2.0) * 0.707;
 AmpsRMS = (VRMS * 1000) / mVperAmp;
 Serial.print(AmpsRMS);
 Serial.print(F(" Amps RMS on "));
 Serial.println(sensorIn);

 if (AmpsRMS > 0.20) {
   if (lastState[sensorIn] == LOW) {
     Serial.println(F("Current rising edge detected. Publishing feed status 1"));
     feed->publish("1");
   }

   lastState[sensorIn] = HIGH;
 } else {
   if (lastState[sensorIn] == HIGH) {
     Serial.println(F("Current falling edge detected. Publishing feed status 0"));
     feed->publish("0");
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
 
