
#include "data_converter.h"
#include <iostream>

using namespace basestation;


const std::string DataConverter::EMPTY = "NONE";
const std::string DataConverter::ERROR = "ERR";
const std::string DataConverter::VALID = "FOUND";
std::regex DataConverter::reg = std::regex("\\b\\d+\\b");


std::string DataConverter::convertToJson(std::string str)
{
    return DataConverter::extractJson(str).toStyledString(); 
}

Json::Value DataConverter::extractJson(std::string str)
{
    if(isEmpty(str)) {
        return DataConverter::getEmptyJson();
    } else if (isError(str)) {
        return DataConverter::getErrorJson();
    } else {
        return DataConverter::parse(str);
    }
}


bool DataConverter::isValid(std::string str)
{
    std::string tmp = trim(str);
    return DataConverter::startsWith(str, VALID);    
}

bool DataConverter::isEmpty(std::string str)
{
    std::string tmp = trim(str);
    return DataConverter::startsWith(str, EMPTY) || tmp.empty();  
}

bool DataConverter::isError(std::string str)
{
    std::string tmp = trim(str);
    return DataConverter::startsWith(str, ERROR);  
}

Json::Value DataConverter::getEmptyJson()
{
    Json::Value root;
    root["beacons"] = "";

    return root;
}
Json::Value DataConverter::getErrorJson()
{
    Json::Value root;
    root["beacons"] = "ERR";

    return root;
}

Json::Value DataConverter::parse(std::string str)
{
    Json::Value root;
    Json::Value vector(Json::arrayValue);

    std::sregex_iterator next(str.begin(), str.end(), reg);
    std::sregex_iterator end;
    

    while (next != end) {

        Json::Value beacon;

        beacon["id_tag"] = getValue(next);
        
        skipForward(next);

        beacon["rssi"] = getValue(next);
        next++;

        vector.append(beacon);
    }

    root["beacons"] = vector;
    return root;
}

void DataConverter::skipForward(std::sregex_iterator &next)
{
    next++;
    next++;
}

std::string DataConverter::getValue(std::sregex_iterator &next)
{
    std::smatch match = *next;
    return match.str();
}

std::string DataConverter::trim(std::string str)
{
    if(str.empty()) return str;
    size_t first = str.find_first_not_of(' ');
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last-first+1));
}

bool DataConverter::startsWith(std::string str, std::string start)
{
    std::size_t found = str.find(start);
    return found != std::string::npos && found == 0;
}
