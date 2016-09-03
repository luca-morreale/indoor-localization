
#include "serial_reader.h"
#include "data_converter.h"

#include <stdexcept>
#include <unistd.h>

using namespace basestation;


SerialReader::SerialReader()
{
	serialFd = serialOpen(DEVICE, BAUD_RATE);
    if(serialFd == -1) {
        throw std::runtime_error("Impossible to open the device!");
    }
}

std::string SerialReader::scan()
{
    std::string data;
    
    serialPrintf(serialFd, "SCANA\n");
    usleep(SLEEP);
    
    while(serialDataAvail(serialFd) > 0) {
        data += (char)serialGetchar(serialFd);
    }

    return data;
}

std::string SerialReader::getJsonScan()
{
	std::string data = scan();
    return DataConverter::convertToJson(data);
}


SerialReader::~SerialReader()
{
	serialClose(serialFd);
}