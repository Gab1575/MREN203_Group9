#include "parse.h"

parse::parse() {
    index = 0; // Initialize here in the constructor instead
    receivedChars[0] = '\0';
}

void parse::startup(){
    clearBuffer();
}

bool parse::run(double& linear_v, double& angular_v) {
    bool successfulParse = false; 
    
    receiveData();
    
    if (newData == true) {
        char* token = strtok(receivedChars, ",");
        
        // Validate header
        if (token != NULL && token[0] == 'V') {
            
            // Grab linear velocity
            token = strtok(NULL, ",");
            if (token != NULL) {
                double temp_linear = atof(token); // Store in a temporary variable
                
                // Grab angular velocity
                token = strtok(NULL, ",");
                if (token != NULL) {
                    double temp_angular = atof(token); // Store in a temporary variable
                    
                    // ATOMIC UPDATE: Only overwrite the actual robot variables 
                    // if we successfully parsed ALL the data in the packet.
                    linear_v = temp_linear;
                    angular_v = temp_angular;
                    successfulParse = true; 
                }
            }
        }
        newData = false; // Always reset flag after reading a packet, success or fail
    }
    
    return successfulParse;
}

void parse::clearBuffer(){
    // 1. Empty the Arduino's hardware RX buffer
    // We read and discard bytes until there are none left.
    while (Serial.available() > 0) {
        Serial.read();
    }

    // 2. Reset our software state machine
    index = 0;
    newData = false;
    receivedChars[0] = '\0'; // Clears the string array by throwing a null terminator at the start
}

void parse::receiveData() {
    // Removed index = 0; from here so the state machine persists
    char endMarker = '\n';
    char rc;

    while(Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[index] = rc;
            index++;
            if (index >= numChars) {
                index = numChars - 1; // Prevent overflow
            }
        }
        else {
            receivedChars[index] = '\0'; // Terminate the string
            index = 0;                   // Reset index ONLY when the packet is complete
            newData = true;
        }
    }
}