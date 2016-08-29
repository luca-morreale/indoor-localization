#include <wiringPi.h>
#include <wiringSerial.h>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

#include <iostream>

#include "data_converter.h"

#define DEVICE "/dev/ttyAMA0"
#define BAUD_RATE 115200

#define ITERATIONS 5


using namespace std;

int main()
{
    int sd = serialOpen(DEVICE, BAUD_RATE);
    if(sd == -1) {
        cout << "Error opening device\n";
        return 1;
    }

    delay(1);

    string measure[ITERATIONS];
    int values[ITERATIONS];
    double avg = 0;
    
    for(int i=0; i<ITERATIONS; i++) {
        serialPrintf(sd, "SCANA\n");
        delay(1);
        
        while(serialDataAvail(sd) > 0) {
            measure[i] += (char)serialGetchar(sd);
        }
        cout << measure[i] << endl;

        Json::Value root = basestation::DataConverter::convertToJson(measure[i]);
        values[i] = root["rssid"].asInt();
        avg += values[i];
    }

    avg /= ITERATIONS;

    for(int i = 0; i <ITERATIONS; i++) {
        cout << values[i] << endl;
    }
    cout << avg << endl;
    
    return 0;
}
