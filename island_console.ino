#include <IRremote.h>
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>

unsigned long lastPing = 0;
unsigned long connectedSince = 0;
unsigned long now = 0;

/************************* Ethernet Client Setup *****************************/
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

//Uncomment the following, and set to a valid ip if you don't have dhcp available.
//IPAddress iotIP (192, 168, 0, 42);
//Uncomment the following, and set to your preference if you don't have automatic dns.
//IPAddress dnsIP (8, 8, 8, 8);
//If you uncommented either of the above lines, make sure to change "Ethernet.begin(mac)" to "Ethernet.begin(mac, iotIP)" or "Ethernet.begin(mac, iotIP, dnsIP)"

#define VERSION_MESSAGE F("Island Console v0.13 24/07/18")

// Adafruit IO setup
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "abc"
#define AIO_KEY         "xyz"

#define SERVER_LISTEN_PORT 80

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
#define COUNTER_LIGHT_PIN 18
#define SINK_LIGHT_PIN 19
#define OUTSIDE_LIGHT_PIN 5
#define KITCHEN_LIGHT_PIN 6
#define ISLAND_LIGHT_PIN 17
#define SIDE_DOOR_PIN 8
#define IR_PIN 9

void(* __resetFunc) (void) = 0; //declare reset function @ address 0

void resetFunc(const __FlashStringHelper* msg) {
  Serial.println(msg);
  Serial.println(F("Resetting in 2 seconds"));
  delay(2000);
  __resetFunc();
}

// Digital Pin states
int sideDoorState = HIGH;
int frontDoorState = HIGH;

int outsideLightState = HIGH;
int kitchenLightState = HIGH;
int sinkLightState = HIGH;
int counterLightState = HIGH;
int islandLightState = HIGH;

//IRrecv irrecv(IR_RECV_PIN);
//decode_results results;

// Controlling the LED strip
IRsend irsend;

EthernetClient client;
EthernetServer server(SERVER_LISTEN_PORT);

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

/****************************** Feeds ***************************************/

#define WILL_FEED AIO_USERNAME "/feeds/nodes.island"
Adafruit_MQTT_Publish lastwill = Adafruit_MQTT_Publish(&mqtt, WILL_FEED);

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish front = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/doors.front");
Adafruit_MQTT_Publish side = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/doors.side");

//Adafruit_MQTT_Subscribe tvtoggle = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.tv");
Adafruit_MQTT_Subscribe ledtoggle = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.led");
Adafruit_MQTT_Subscribe kitchenlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.kitchenlight");
Adafruit_MQTT_Subscribe outsidelight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.outsidelight");
Adafruit_MQTT_Subscribe islandlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.islandlight");
Adafruit_MQTT_Subscribe counterlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.counterlight");
Adafruit_MQTT_Subscribe sinklight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.sinklight");

// Possibly as
//  #define TOGGLE_TV "1"
//  #define TOGGLE_SOUNDBAR "2"
//  Adafruit_MQTT_Subscribe devicetoggle = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/devicetoggle");
//

/*************************** Sketch Code ************************************/

void setup() {

  pinMode(SIDE_DOOR_PIN, INPUT_PULLUP);
  pinMode(FRONT_DOOR_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, OUTPUT);

  pinMode(OUTSIDE_LIGHT_PIN, OUTPUT);
  pinMode(KITCHEN_LIGHT_PIN, OUTPUT);
  pinMode(ISLAND_LIGHT_PIN, OUTPUT);
  pinMode(COUNTER_LIGHT_PIN, OUTPUT);
  pinMode(SINK_LIGHT_PIN, OUTPUT);

  digitalWrite(OUTSIDE_LIGHT_PIN, HIGH);
  digitalWrite(KITCHEN_LIGHT_PIN, HIGH);
  digitalWrite(ISLAND_LIGHT_PIN, HIGH);
  digitalWrite(COUNTER_LIGHT_PIN, HIGH);
  digitalWrite(SINK_LIGHT_PIN, HIGH);

  Serial.begin(115200);

  Serial.println(VERSION_MESSAGE);

  // Initialise the Client
  Serial.println(F("Joining the network..."));
  Ethernet.begin(mac);
  delay(1000); //give the ethernet a second to initialize
  Serial.print("HTTP server listening on ");
  Serial.println(Ethernet.localIP());


  //MQTT_connect();
  Serial.println("MQTT subscribe");
  //mqtt.subscribe(&tvtoggle);
  mqtt.subscribe(&counterlight);
  mqtt.subscribe(&sinklight);
  mqtt.subscribe(&kitchenlight);
  mqtt.subscribe(&islandlight);
  mqtt.subscribe(&outsidelight);
  mqtt.subscribe(&ledtoggle);

  mqtt.will(WILL_FEED, "0");

  //mqtt.subscribe(&soundbartoggle);

  Serial.println("Enable IR");
  //irrecv.enableIRIn();
  //irrecv.blink13(true);
}

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  now = millis();
  Ethernet.maintain();
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    Serial.println("Incoming message");
    //irrecv.enableIRIn();
    //irrecv.resume();

   // if (subscription == &tvtoggle) {
     // Serial.println("msg: TOGGLE TV");
     // irsend.sendRC5(0xc, 4);
    //} else
    if (subscription == &ledtoggle) {
      char* command = (char*)subscription->lastread;
      command[0] = tolower(command[0]);

      Serial.print("msg: TOGGLE LED ");
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

      if (strstr(command, "brighter") != NULL || strstr(command, "Brighter") != NULL) {
        irsend.sendNEC(Brighter, 32);
        delay(40);
      } else if (strstr(command, "dimmer") != NULL || strstr(command, "Dimmer") != NULL) {
        irsend.sendNEC(Dimmer, 32);
        delay(40);
      }

      if (strstr(command, "fade") != NULL || strstr(command, "Fade") != NULL) {
        irsend.sendNEC(Fade7, 32);
        delay(40);
      } else if (strstr(command, "strobe") != NULL || strstr(command, "Strobe") != NULL) {
        irsend.sendNEC(Jump7, 32);
        delay(40);
      }

      if (strstr(command, "slow") != NULL || strstr(command, "Slow") != NULL) {
        irsend.sendNEC(Slow, 32);
        delay(40);
      } else if (strstr(command, "fast") != NULL || strstr(command, "Fast") != NULL) {
        irsend.sendNEC(Fast, 32);
        delay(40);
      }

      if (strstr(command, "brightest")  != NULL || strstr(command, "Brightest")  != NULL) {
        for (int i = 0; i < 13; ++i) {
          irsend.sendNEC(Brighter, 32);
          delay(40);
        }
      }

      if (strstr(command, "dimmest") != NULL || strstr(command, "Dimmest")  != NULL) {
        for (int i = 0; i < 13; ++i) {
          irsend.sendNEC(Dimmer, 32);
          delay(40);
        }
      }

      //irsend.sendNEC(0xFF02FD, 32);
    } else if (subscription == &outsidelight) {

      Serial.println(F("msg: OUTSIDE LIGHT"));
      outsideLightState = (outsideLightState == HIGH ? LOW : HIGH);
      digitalWrite(OUTSIDE_LIGHT_PIN, outsideLightState);


    } else if (subscription == &kitchenlight) {

      Serial.println(F("msg: KITCHEN LIGHT"));
      kitchenLightState = (kitchenLightState == HIGH ? LOW : HIGH);
      digitalWrite(KITCHEN_LIGHT_PIN, kitchenLightState);


    } else if (subscription == &islandlight) {

      Serial.println(F("msg: ISLAND LIGHT"));
      islandLightState = (islandLightState == HIGH ? LOW : HIGH);
      digitalWrite(ISLAND_LIGHT_PIN, islandLightState);


    } else if (subscription == &counterlight) {

      Serial.println(F("msg: COUNTER LIGHT"));
      counterLightState = (counterLightState == HIGH ? LOW : HIGH);
      digitalWrite(COUNTER_LIGHT_PIN, counterLightState);


    } else if (subscription == &sinklight) {

      Serial.println(F("msg: SINK LIGHT"));
      sinkLightState = (sinkLightState == HIGH ? LOW : HIGH);
      digitalWrite(SINK_LIGHT_PIN, sinkLightState);
    }

    //else if (subscription == &soundbartoggle) {
     // Serial.println("msg: TOGGLE SOUNDBAR");
     // irsend.sendRC6(0x150C, 16);
   // }
  }

  detectEdge(FRONT_DOOR_PIN, &front);
  detectEdge(SIDE_DOOR_PIN, &side);

  handleHttpClientRequest();

  //IR_decode();
  MQTT_ping();

  delay(1000);
}


int lastState[25] = {0};

void detectEdge(int pin, Adafruit_MQTT_Publish* feed) {
  int state = digitalRead(pin);
  if (state != lastState[pin]) {
    Serial.print("Publishing state change on pin ");
    Serial.print(pin);
    if (state == HIGH) {
      Serial.println(", high");
      feed->publish("1");
    } else {
      Serial.println(", low");
      feed->publish("0");
    }
  }

  lastState[pin] = state;
}
/*
void IR_decode() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    switch (results.decode_type) {
        case NEC: Serial.println("NEC"); break ;
        case SONY: Serial.println("SONY"); break ;
        case RC5: Serial.println("RC5"); break ;
        case RC6: Serial.println("RC6"); break ;
        case DISH: Serial.println("DISH"); break ;
        case SHARP: Serial.println("SHARP"); break ;
        case JVC: Serial.println("JVC"); break ;
        case SANYO: Serial.println("SANYO"); break ;
        case MITSUBISHI: Serial.println("MITSUBISHI"); break ;
        case SAMSUNG: Serial.println("SAMSUNG"); break ;
        case LG: Serial.println("LG"); break ;
        case WHYNTER: Serial.println("WHYNTER"); break ;
        case AIWA_RC_T501: Serial.println("AIWA_RC_T501"); break ;
        case PANASONIC: Serial.println("PANASONIC"); break ;
        case DENON: Serial.println("DENON"); break ;
      default:
        case UNKNOWN: Serial.println("UNKNOWN"); break ;
    }

    irrecv.resume();
    irrecv.enableIRIn();
  }
}
*/

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

          client.print(F("<h1>"));
          client.print(VERSION_MESSAGE);
          client.println(F("</h1>"));
          client.print(F("<br />Outside light is "));
          client.println(outsideLightState);
          client.print(F("<br />Kitchen light is "));
          client.println(kitchenLightState);
          client.print(F("<br />Counter light is "));
          client.println(counterLightState);
          client.print(F("<br />Island light is "));
          client.println(islandLightState);
          client.print(F("<br />Sink light is "));
          client.println(sinkLightState);
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
