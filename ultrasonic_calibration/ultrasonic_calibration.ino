const int echoPin = A5;
const int trigPin = A4;

// running sums for linear regression
long nPoints = 0;
double sumDur = 0;
double sumDist = 0;
double sumDurDist = 0;
double sumDur2 = 0;

void setup() {
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  Serial.begin(9600);
  // print header for CSV and instructions
  Serial.println("pulse duration (microseconds),distance (cm)");
  Serial.println("Enter a known distance (cm) followed by newline to record a measurement");
}

long measureDuration() {
  // Send a short low pulse to ensure a clean high one.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Send a ten-microsecond high pulse.
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  return pulseIn(echoPin, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    double dist = Serial.parseFloat();
    if (dist > 0) {
      long duration = measureDuration();
      // output CSV line
      Serial.print(duration);
      Serial.print(",");
      Serial.println(dist);

      // update sums
      nPoints++;
      sumDur += duration;
      sumDist += dist;
      sumDurDist += duration * dist;
      sumDur2 += (double)duration * duration;

      // compute linear regression y = m x + c where y=distance, x=duration
      double m = 0;
      double c = 0;
      if (nPoints > 1) {
        double numerator = nPoints * sumDurDist - sumDur * sumDist;
        double denominator = nPoints * sumDur2 - sumDur * sumDur;
        if (denominator != 0) {
          m = numerator / denominator;
        }
        c = (sumDist - m * sumDur) / nPoints;
      }
      Serial.print("Regression: y = ");
      Serial.print(m, 6);
      Serial.print(" x + ");
      Serial.println(c, 6);
      Serial.println("------");
    }
    // flush any leftover input to avoid repeated reads
    while (Serial.available() > 0) Serial.read();
  }
}
