#ifndef DATAPARSER_HPP
#define DATAPARSER_HPP

#include <string>
#include <vector>
#include <sstream>

class DataParser {
public:
float gyroZ = 0.0; 
float accelX = 0.0; 
float accelY = 0.0; 
float accelZ = 0.0; 
float omega_L = 0.0; 
float omega_R = 0.0;

void parse(const std::string& line) {
    //create a stream from the serial string
    std::stringstream ss(line);
    std::string chunk;
    std::vector<float> values;

    //split the string from commas 
    while (std::getline(ss, chunk, ',')) {
        try {
            values.push_back(std::stof(chunk)); // Convert to float and store
        } catch (...) {
            continue;
        }
    }
    //If all 6 data values are present, update class variables 
    if (values.size() == 6) {
        gyroZ = values[0];
        accelX = values[1];
        accelY = values[2];
        accelZ = values[3];
        omega_L = values[4];
        omega_R = values[5];
        }
    }
};

#endif //DATAPARSER_HPP