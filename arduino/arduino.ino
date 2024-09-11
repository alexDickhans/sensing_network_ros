/*



	Based on the Minimal Esp32 Websockets Client example from Gil Maimon
*/


#include <ArduinoWebsockets.h>
#include <CapacitiveSensorR4.h>
#include <WiFi.h>

// CONFIGURATION
const char* ssid = "HIROLab2";                                        //Enter SSID
const char* password = "HIROlab322";                            //Enter Password
const char* websockets_server_host = "ws://192.168.0.184:8765/";  //Enter server adress

CapacitiveSensor sensor(4,2);

uint32_t lastSend = 0;

uint16_t touchPoints[] = {26, 27, 28, 29};
uint32_t minValue = 0;

uint32_t numSamples = 200;

using namespace websockets;

void onMessageCallback(WebsocketsMessage message) {
  Serial.print("Got Message: ");
  Serial.println(message.data());
}

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

WebsocketsClient client;
void setup() {

  for (size_t i = 0; i < 2000; i++) {
    minValue = std::max(minValue, sensor.capacitiveSensorRaw(numSamples));
  }

  minValue = minValue * 10500000000000 / 10000000000000;

  Serial.begin(115200);
  // Connect to wifi
  WiFi.begin(ssid, password);

  Serial.println("start");

  // Wait some time to connect to wifi
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
    Serial.print(".");
    delay(1000);
  }

  // run callback when messages are received
  client.onMessage(onMessageCallback);

  // run callback when events are occuring
  client.onEvent(onEventsCallback);

  // Connect to server
  while (!client.connect(websockets_server_host)) {
    Serial.println("again");
    delay(500);
  }
}

void loop() {
  client.poll();
  delay(1);

  if (client.available() && millis() - lastSend > 9) {

    lastSend = millis();

    // Send a message
    auto reading = sensor.capacitiveSensorRaw(numSamples);

    Serial.println(reading);
    
    bool binaryReading = minValue < reading;

    for (size_t i = 0; i < sizeof(touchPoints) / sizeof(touchPoints[0]); i++) {
      client.send(String(touchPoints[i]) + "," + String(binaryReading));
    }
  } else if (!client.available() && millis() % 50) {
    client.connect(websockets_server_host);
  }
}
