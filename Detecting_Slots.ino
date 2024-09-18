#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "RJ";
const char* password = "1234567898";
const char* mqtt_server = "ddc395f077c4497aa275056e4b727fb2.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "Omar Alsharqawi";
const char* mqtt_pass = "Im14Im..&&";

const char* publishTopic = "parking/status";

WiFiClient espClient;
PubSubClient client(espClient);

const int irPins[6] = {2, 3, 4, 5, 6, 7};

int slotStatus[6] = {0, 0, 0, 0, 0, 0};
int freeSlots = 6;

void setup() {
  Serial.begin(115200);
  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  delay(2000);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  checkSlots();
  sendSlotStatus();
  delay(1000);
}

void setupWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe(publishTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void checkSlots() {
  for (int i = 0; i < 6; i++) {
    int irStatus = digitalRead(irPins[i]);
    if (irStatus == HIGH && slotStatus[i] == 0) {
      slotStatus[i] = 1;
      Serial.println("Slot " + String(i + 1) + " is now FULL");
    } else if (irStatus == LOW && slotStatus[i] == 1) {
      slotStatus[i] = 0;
      Serial.println("Slot " + String(i + 1) + " is now EMPTY");
    }
  }
  freeSlots = 0;
  for (int i = 0; i < 6; i++) {
    if (slotStatus[i] == 0) {
      freeSlots++;
    }
  }
}

void sendSlotStatus() {
  String message = "Parking Status:\n";
  for (int i = 0; i < 6; i++) {
    message += "Slot " + String(i + 1) + ": " + (slotStatus[i] == 1 ? "FULL" : "EMPTY") + "\n";
  }
  message += "Free Slots: " + String(freeSlots);
  client.publish(publishTopic, message.c_str());
  Serial.println("Sent message: " + message);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
