#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <CapacitiveSensorR4.h>
#include "credentials.hpp"

using namespace websockets;

// CONFIGURATION
const char* ssid = "HIROLab2";                                        //Enter SSID
const char* password = "HIROlab322";                            //Enter Password
const char* websockets_server_host = "ws://192.168.0.152:8765/";  //Enter server adress

CapacitiveSensor sensor(4, 2);

WebsocketsClient client;

struct _TouchPointRange {
  uint16_t index;
  uint16_t min;
  uint16_t max;
} typedef TouchPointRange;

TouchPointRange* touchRanges;
int numTouchPoints;
int startIndex;
int samples = 200;

void onEventsCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("Connnection Opened");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("Connnection Closed");
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Got a Ping!");
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Got a Pong!");
  }
}

void print(String data) {
  client.send(data.c_str());
  Serial.println(data);
}

void setup() {
  Serial.begin(115200);
  // Connect to wifi
  WiFi.begin(ssid, password);

  Serial.println("start");

  // Wait some time to connect to wifi
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
    Serial.print(".");
    delay(1000);
  }

  // // run callback when messages are received
  // client.onMessage(onMessageCallback);

  // run callback when events are occuring
  client.onEvent(onEventsCallback);

  // Connect to server
  while (!client.connect(websockets_server_host)) {
    Serial.println("again");
    delay(100);
  }

  print("How many touch points?");

  // numTouchPoints = atoi(client.readBlocking().c_str());

  // if (numTouchPoints < 1) {
  //   print("Too few touch points, restart");
  //   ESP.restart();
  // }

  // print("You selected " + String(numTouchPoints) + " touch points.\n\nWhat should the start index be?\n");


  // startIndex = atoi(client.readBlocking().c_str());

  // if (startIndex < 0) {
  //   print("Index must be positive, restart\n\n");
  //   ESP.restart();
  // }

  // print("You selected " + String(startIndex) + " start index.\n\nHow many samples should it take?\n");

  // samples = atoi(client.readBlocking().c_str());

  // if (samples < 5) {
  //   print("Sample count must be 5 or greater, restart\n\n");
  //   ESP.restart();
  // }

  // print("You selected " + String(samples) + " samples.\n\n");

  // touchRanges = (TouchPointRange*) calloc(numTouchPoints, sizeof(TouchPointRange));

  // for (size_t i = 0; i < numTouchPoints; i++) {
  //   print("Touch point " + String(startIndex + i) + " for 5 seconds\n");
  //   delay(3000);

  //   uint32_t startTime = millis();

  //   uint32_t sample = sensor.capacitiveSensorRaw(samples);

  //   uint32_t highValue = sample;
  //   uint32_t lowValue = sample;

  //   while (millis() - startTime < 3000) {
  //     sample = sensor.capacitiveSensorRaw(samples);

  //     if (sample == -2) {
  //       // nothing
  //     } else if (sample > highValue) {
  //       highValue = sample;
  //     } else if (sample < lowValue) {
  //       lowValue = sample;
  //     }
  //   }

  //   print("High value: " + String(highValue) + ", Low value: " + String(lowValue) + "\n");

  //   touchRanges[i] = {i + startIndex, lowValue, highValue};
  // }

  // print("\n\n\n\TouchPointRange touchRanges[] = {\n");

  // for (size_t i = 0; i < numTouchPoints; i++) {
  //   print("{" + String(touchRanges[i].index) + ", " + String(touchRanges[i].min) + ", " + String(touchRanges[i].max) + "},\n");
  // }

  // print("};\n");

  // delay(5000);
}

void loop() {
  delay(10);

  auto reading = sensor.capacitiveSensorRaw(samples);

  print(String(reading));

  // for (size_t i = 0; i < numTouchPoints; i++) {
  //   bool touchReading = reading > touchRanges[i].min && reading < touchRanges[i].max;
  //   print(String(touchRanges[i].index) + ", " + String(touchReading));
  // }

  client.poll();
}
