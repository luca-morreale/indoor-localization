#ifndef BASESTATION_SERIALREADER_H
#define BASESTATION_SERIALREADER_H

#include <wiringPi.h>
#include <wiringSerial.h>

#include <string>

namespace basestation {

	#define SLEEP 401000 // 401 ms

    #define DEVICE "/dev/ttyAMA0"
    #define BAUD_RATE 115200

	class SerialReader {
	public:
        SerialReader();
        ~SerialReader();

        virtual std::string scan();
        virtual std::string getJsonScan();

    private:
        int serialFd;
    };
}


#endif /* BASESTATION_SERIALREADER_H */