// Define pins for the first ultrasonic sensor

#define TRIG_PIN_1 15   // TRIG pin for sensor 1 
#define ECHO_PIN_1 2  // ECHO pin for sensor 1 

// Define pins for the second ultrasonic sensor
#define TRIG_PIN_2 5   // TRIG pin for sensor 2
#define ECHO_PIN_2 18   // ECHO pin for sensor 2
// Define pins for the third ultrasonic sensor

#define TRIG_PIN_3 33   // TRIG pin for sensor 3
#define ECHO_PIN_3 32   // ECHO pin for sensor 3

// Define pins for the fourth ultrasonic sensor
#define TRIG_PIN_4 27   // TRIG pin for sensor 3
#define ECHO_PIN_4 26   // ECHO pin for sensor 4


void setup() {
  // Initialize serial communication for displaying sensor data
  Serial.begin(9600);

  // Setup the pins for both sensors
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);
  pinMode(TRIG_PIN_4, OUTPUT);
  pinMode(ECHO_PIN_4, INPUT);
}

void loop() {
  // First call: Read Sensor 1
  float distance_1 = getDistance(TRIG_PIN_1, ECHO_PIN_1);
  Serial.print("Sensor 1 Distance: ");
  Serial.print(distance_1);
  Serial.println(" cm");

  // Second call: Read 6 2
  float distance_2 = getDistance(TRIG_PIN_2, ECHO_PIN_2);
  Serial.print("Sensor 2 Distance: ");
  Serial.print(distance_2);
  Serial.println(" cm");

    // third call: Read Sensor 1
  float distance_3 = getDistance(TRIG_PIN_3, ECHO_PIN_3);
  Serial.print("Sensor 3 Distance: ");
  Serial.print(distance_3);
  Serial.println(" cm");

  // forth call: Read 6 2
  float distance_4 = getDistance(TRIG_PIN_4, ECHO_PIN_4);
  Serial.print("Sensor 4 Distance: ");
  Serial.print(distance_4);
  Serial.println(" cm");


  // Add delay to avoid rapid cycling between readings
  delay(1000);  // 1 second between each full cycle
}

// Function to get the distance from the ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  // Clear the trigPin by setting it LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by setting the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin and calculate the time it took for the sound wave to bounce back
  long duration = pulseIn(echoPin, HIGH, 25000); // Timeout after 25ms for max range

  // Calculate the distance (duration / 2 * speed of sound)
  if (duration == 0) {
    return -1; // No reading, return invalid distance
  }
  float distance = duration * 0.034 / 2;
  return distance;
}
