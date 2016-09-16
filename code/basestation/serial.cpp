#include <wiringPi.h>
#include <wiringSerial.h>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

#include <iostream>
#include <vector>
#include <unistd.h>

#include "data_converter.h"

#define DEVICE "/dev/ttyAMA0"
#define BAUD_RATE 115200

#define ITERATIONS 5
#define SLEEP 401000 // 401 ms


using namespace std;

void fillRSSIValues(int sd, vector<int> &values);
double averageRSSID(vector<int> &nums);
int majorityElement(vector<int> &nums);

int main()
{
    int sd = serialOpen(DEVICE, BAUD_RATE);
    if(sd == -1) {
        cout << "Error opening device\n";
        return 1;
    }

    usleep(SLEEP);
    
    vector<int> values(ITERATIONS);
    fillRSSIValues(sd, values);

    for(int i = 0; i <ITERATIONS; i++) {
        cout << "value-" << i << ": " << values[i] << endl;
    }
    cout << "average: " << averageRSSID(values) << endl;
    cout << "majority element:" << majorityElement(values) << endl;
    
    return 0;
}

void fillRSSIValues(int sd, vector<int> &values)
{
    std::string measure;
    
    for(int i=0; i<ITERATIONS; i++) {
        serialFlush(sd);
        serialPrintf(sd, "SCANA\n");
        usleep(SLEEP);
        
        while(serialDataAvail(sd) > 0) {
            measure += (char)serialGetchar(sd);
        }
        cout << measure << endl;

        Json::Value root = basestation::DataConverter::extractJson(measure);

        values[i] = root["beacons"][0]["rssid"].asInt();
    }
}

double averageRSSID(vector<int> &nums)
{
    int sum = 0;
    for (int n : nums) {
        sum += n;
    }
    return sum / nums.size();
}

int majorityElement(vector<int> &nums)
{
    int candidate = nums[0], counter = 0;
    for (int n: nums) {
        if (counter == 0) {
            candidate = n;
            counter = 1;
        } else if (candidate == n) {
            counter++;
        } else {
            counter--;
        }
    }
    return candidate;
}