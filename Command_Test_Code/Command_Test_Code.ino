
double motor1Angle = 0.0;
double motor2Angle = 0.0;

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming data
    char message = Serial.read();

    // Buffer to hold the command string
    char command[20];
    int index = 0;
    delay(500);
    // Read until a newline character is received
    while (data != '\n' && index <= sizeof(command) - 1) {
      command[index] = message;
      index++;
      if (Serial.available() > 0) {
        message = Serial.read();
      } else {
        break;
      }
    }
    // Null-terminate the command string
    command[index] = '\0';
    Serial.println(command);
    char* secCom;
    Serial.println(strtod(command, &secCom));
    Serial.println(strtod(secCom, NULL));

  }
}