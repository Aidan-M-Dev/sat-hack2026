const int echoPin = A5;
const int trigPin = A4;
void setup() {
pinMode(echoPin, INPUT);
pinMode(trigPin, OUTPUT);
Serial.begin(9600);
}
void loop() {
// Send a short low pulse to ensure a clean high one.
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Send a ten-second high pulse.
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

const long duration = pulseIn(echoPin, HIGH);
Serial.print("Period (microseconds): ");
Serial.println(duration);
}