const int redPins[] = {2, 5, 8};    // Red LED pins for each road
const int yellowPins[] = {3, 6, 9}; // Yellow LED pins for each road
const int greenPins[] = {4, 7, 10}; // Green LED pins for each road

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 3; i++) {
    pinMode(redPins[i], OUTPUT);
    pinMode(yellowPins[i], OUTPUT);
    pinMode(greenPins[i], OUTPUT);
    digitalWrite(redPins[i], HIGH);    // Initialize with red light
    digitalWrite(yellowPins[i], LOW);
    digitalWrite(greenPins[i], LOW);
  }
}

void loop() {
  if (Serial.available() > 0) {
    String signal = Serial.readStringUntil('\n');
    int roadIndex = signal.charAt(1) - '1'; // Convert signal to road index (0, 1, 2)
    if (signal.charAt(0) == 'G') {
      setGreenLight(roadIndex);
    } else if (signal.charAt(0) == 'Y') {
      setYellowLight(roadIndex);
    } else if (signal.charAt(0) == 'R') {
      setRedLight(roadIndex);
    }
  }
}

void setGreenLight(int roadIndex) {
  for (int i = 0; i < 3; i++) {
    if (i == roadIndex) {
      digitalWrite(greenPins[i], HIGH);
      digitalWrite(yellowPins[i], LOW);
      digitalWrite(redPins[i], LOW);
    } else {
      digitalWrite(greenPins[i], LOW);
      digitalWrite(yellowPins[i], LOW);
      digitalWrite(redPins[i], HIGH);
    }
  }
}

void setYellowLight(int roadIndex) {
  digitalWrite(greenPins[roadIndex], LOW);
  digitalWrite(yellowPins[roadIndex], HIGH);
  digitalWrite(redPins[roadIndex], LOW);
}

void setRedLight(int roadIndex) {
  digitalWrite(greenPins[roadIndex], LOW);
  digitalWrite(yellowPins[roadIndex], LOW);
  digitalWrite(redPins[roadIndex], HIGH);
}