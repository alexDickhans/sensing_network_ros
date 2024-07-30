/*



	Based on the Minimal Esp32 Websockets Client example from Gil Maimon
*/


#include <ArduinoWebsockets.h>
#include <CapacitiveSensorR4.h>
#include <WiFi.h>

// CONFIGURATION
const char* ssid = "HIROLab2";                                        //Enter SSID
const char* password = "HIROlab322";                            //Enter Password
const char* websockets_server_host = "ws://192.168.0.221:8765/";  //Enter server adress

using namespace websockets;

CapacitiveSensor sensor(4, 2);

struct _TouchPointRange {
  uint16_t index;
  uint16_t min;
  uint16_t max;
} typedef TouchPointRange;

TouchPointRange touchRanges[] = {
  { 0, 1440, 1758 },
  { 1, 1423, 1446 },
};

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

  if (client.available() && millis() % 10 == 0) {
    // Send a message
    auto reading = sensor.capacitiveSensorRaw(200);

    for (size_t i = 0; i < sizeof(touchRanges) / sizeof(touchRanges[0]); i++) {
      bool touchReading = reading > touchRanges[i].min && reading < touchRanges[i].max;
      client.send(String(touchRanges[i].index) + "," + String(touchReading));
    }

  } else if (!client.available() && millis() % 50) {
    client.connect(websockets_server_host);
  }
}
