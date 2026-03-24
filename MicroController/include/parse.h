#ifndef PARSE_H
#define PARSE_H

#include <Arduino.h>

class parse {
public:
    parse();
    bool run(double& linear_v, double& angular_v); 

private:
    static const byte numChars = 32;
    char receivedChars[numChars];    
    boolean newData = false;
    byte index;                    

    void receiveData();
};

#endif // PARSE_H