#include <Arduino.h>


// the setup function runs once when you press reset or power the board
void setup() {
  
  Serial.begin(115200);
  Serial.println("Boot starting.");

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(5, OUTPUT);
  pinMode(2, OUTPUT);

  Serial.println("Boot complete.");
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(5, HIGH);
  digitalWrite(2, HIGH);
  delay(1000);                       // wait for a second
  digitalWrite(5, LOW);
  digitalWrite(2, LOW);
  delay(1000);                       // wait for a second
}
