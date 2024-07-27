/*
	Minimal Esp32 Websockets Client

	This sketch:
        1. Connects to a WiFi network
        2. Connects to a Websockets server
        3. Sends the websockets server a message ("Hello Server")
        4. Sends the websocket server a "ping"
        5. Prints all incoming messages while the connection is open

    NOTE:
    The sketch dosen't check or indicate about errors while connecting to
    WiFi or to the websockets server. For full example you might want
    to try the example named "Esp32-Client".

	Hardware:
        For this sketch you only need an ESP8266 board.

	Created 15/02/2019
	By Gil Maimon
	https://github.com/gilmaimon/ArduinoWebsockets

*/

#include <ArduinoWebsockets.h>
#include <WiFi.h>

const char* ssid = "Home";                                       //Enter SSID
const char* password = "dickhans2845";                           //Enter Password
const char* websockets_server_host = "ws://192.168.0.137:8765/";  //Enter server adress

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
  while(!client.connect(websockets_server_host)) {
    Serial.println("again");
    delay(500);
  }
}

void loop() {
  client.poll();
  delay(1);

  if (client.available()) {
    // Send a message
    client.send(String(millis() % 5) + "," + String(random(10)));
  } else {
    client.connect(websockets_server_host);
  }
}
