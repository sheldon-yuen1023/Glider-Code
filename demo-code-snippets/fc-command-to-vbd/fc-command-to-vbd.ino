#include <Arduino.h>



// Create an instance of HardwareSerial for communication with the VBD

HardwareSerial vbdSerial(2);



// Define the UART pins for the Flight Controller Simulator.

// These pins are cross-connected to the VBD’s UART pins such that:

//   Flight Controller TX (to VBD RX) uses GPIO 18  

//   Flight Controller RX (from VBD TX) uses GPIO 17

#define FC_UART_RX_PIN 18  

#define FC_UART_TX_PIN 17  



void setup() {

  // Initialize the USB Serial Monitor for PC communication.

  Serial.begin(115200);

  while (!Serial) {

    ; // Wait for Serial to connect

  }

  

  Serial.println("Flight Controller Simulator Initialised");

  Serial.println("Enter a number (1-7) to send a command to the VBD:");

  Serial.println("1 -> INITIALISE");

  Serial.println("2 -> START MISSION (e.g., 3 cycles)");

  Serial.println("3 -> MOVE_TO (e.g., target rotations 7)");

  Serial.println("4 -> SET_LINEAR_LIMIT (e.g., distance 15.5)");

  Serial.println("5 -> STOP");

  Serial.println("6 -> TARGET_DEPTH_REACHED");

  Serial.println("7 -> TARGET_DEPTH_REACHED2");



  // Initialize the UART used to communicate with the VBD.

  vbdSerial.begin(115200, SERIAL_8N1, FC_UART_RX_PIN, FC_UART_TX_PIN);

}



void loop() {

  // Check for input from the PC Serial Monitor.

  if (Serial.available() > 0) {

    char input = Serial.read();

    // Flush any extra incoming characters.

    while (Serial.available() > 0) {

      Serial.read();

    }

    

    // Depending on the input number, send a corresponding command message.

    switch (input) {

      case '1':

        vbdSerial.println("INITIALISE");

        Serial.println("Sent: INITIALISE");

        break;

      case '2':

        vbdSerial.println("START MISSION 3");

        Serial.println("Sent: START MISSION 3");

        break;

      case '3':

        vbdSerial.println("MOVE_TO 7");

        Serial.println("Sent: MOVE_TO 7");

        break;

      case '4':

        vbdSerial.println("SET_LINEAR_LIMIT 15.5");

        Serial.println("Sent: SET_LINEAR_LIMIT 15.5");

        break;

      case '5':

        vbdSerial.println("STOP");

        Serial.println("Sent: STOP");

        break;

      case '6':

        vbdSerial.println("TARGET_DEPTH_REACHED");

        Serial.println("Sent: TARGET_DEPTH_REACHED");

        break;

      case '7':

        vbdSerial.println("TARGET_DEPTH_REACHED2");

        Serial.println("Sent: TARGET_DEPTH_REACHED2");

        break;

      default:

        Serial.println("Unknown command. Please enter a number between 1 and 7.");

        break;

    }

  }



  // Optionally, read any responses from the VBD and print them to the Serial Monitor.

  if (vbdSerial.available()) {

    String response = vbdSerial.readStringUntil('\n');

    Serial.print("Received from VBD: ");

    Serial.println(response);

  }

  

  delay(10); // Short delay for loop timing

}