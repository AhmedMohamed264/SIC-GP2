#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// WiFi credentials
const char* ssid = "Sharq";          
const char* password = "1234567890";  

// MQTT broker credentials
const char* mqttServer = "c602d4e93f274a9e93bf8ba3395ef9ed.s1.eu.hivemq.cloud";
const int mqttPort = 8883;
const char* mqttUser = "omar2003";
const char* mqttPassword = "12345678@Nu";
const char* mqttTopic = "gate";  

// Root CA certificate
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";
// WiFi and MQTT clients
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Define sensor pins
const int ldr_pins[2] = {34, 35}; 
const int gateServoPin = 18;      
Servo gateServo;                  

void setup() {
  Serial.begin(115200);
  delay(100);

  connectToWiFi();
  espClient.setCACert(root_ca);
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  connectToMQTT();

  for (int i = 0; i < 2; i++) {
    pinMode(ldr_pins[i], INPUT);
  }
  gateServo.attach(gateServoPin);
  gateServo.write(0); // Start with gate closed
}

void loop() {
  if (!client.connected()) {
    connectToMQTT();
  }
  client.loop();
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("connected");
      client.subscribe(mqttTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\nMessage arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strncmp((char*)payload, "open", length) == 0) {
    handleGateOperation();
  }
}

void handleGateOperation() {
  Serial.println("Handling gate operation...");

  // Read LDR values
  int ldrValue1 = analogRead(ldr_pins[0]);
  int ldrValue2 = analogRead(ldr_pins[1]);

  Serial.print("LDR Value 1 (Outside): ");
  Serial.println(ldrValue1);
  Serial.print("LDR Value 2 (Inside): ");
  Serial.println(ldrValue2);
  
  if (ldrValue1 < 3000 || ldrValue2 < 3000) {
    Serial.println("Object detected, opening gate...");
    for (int angle = 0; angle <= 90; angle += 10) {
        gateServo.write(angle);  // Open gate
        delay(100);  // Wait for the servo to move
      }

         const char* topic = "parking";
         const char* message = "5";
         client.publish(topic, message);
 // Open gate
    delay(3000);  // Allow gate to open

    // Check for detection to close gate
    while (true) {
      int currentLdr1 = analogRead(ldr_pins[0]);
      int currentLdr2 = analogRead(ldr_pins[1]);
      Serial.print("Current LDR 1: ");
      Serial.println(currentLdr1);
      Serial.print("Current LDR 2: ");
      Serial.println(currentLdr2);

      if (currentLdr1 > 3000 && currentLdr2 > 3000) {
        Serial.println("No object detected, closing gate...");
         for (int angle = 90; angle >= 0; angle -= 10) {
            gateServo.write(angle);  // Close gate
            delay(100);  // Wait for the servo to move
          }  // Close gate
        delay(2000);  // Allow gate to close
        break;
      }
      delay(500);
    }
  } else {
    Serial.println("No objects detected, keeping gate closed.");
  }
}
