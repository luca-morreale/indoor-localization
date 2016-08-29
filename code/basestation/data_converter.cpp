
#include "data_converter.h"

using namespace basestation;


const std::string DataConverter::EMPTY = "NONE";
const std::string DataConverter::ERROR = "ERR";
const std::string DataConverter::VALID = "FOUND";
std::regex DataConverter::reg = std::regex("<\\w+>");


std::string DataConverter::convertToJson(std::string str)
{

    if(isEmpty(str)) {
        return DataConverter::getEmptyJson().toStyledString();
    } else if (isError(str)) {
        return DataConverter::getErrorJson().toStyledString();
    } else {
        return DataConverter::parse(str).toStyledString();
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
    return DataConverter::startsWith(str, EMPTY);  
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
    VectorString data;
    std::smatch match;

    Json::Value root;
    Json::Value vec(Json::arrayValue);

    //std::regex_search(str, self_regex)
    std::regex_match(str, match, reg);

    for(int i = 0; i < match.size(); i += 3) {
        Json::Value beacon;
        beacon["id_tag"] = match.str(i);
        beacon["rssid"] = match.str(i+2);

        vec.append(beacon);
    }

    root["beacons"] = vec;

    return root;
}

std::string DataConverter::trim(std::string str)
{
    size_t first = str.find_first_not_of(' ');
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last-first+1));
}

bool DataConverter::startsWith(std::string str, std::string start)
{
    std::size_t found = str.find(start);
    return found != std::string::npos && found == 0;
}
