
#include "data_converter.h"
#include <string>
#include <algorithm>
#include <iterator>
#include <regex>
#include <vector>

const std::string DataConverter::EMPTY = "NONE";
const std::string DataConverter::ERROR = "ERR";
const std::string DataConverter::VALID = "FOUND";

/*
    FOUND,<id tag>, <battery>,
    <RSSI 125>, <id tag>,
    <battery>, <RSSI 125> ...
    DONE - NONE
*/



std::string DataConverter::convertToJson(std::string str)
{
    if(isEmpty(str)) {

    } else if (isError(str)) {

    } else {
        
        std::copy(std::sregex_token_iterator(str.begin(), str.end(), "\\s+", -1),
              std::sregex_token_iterator(),
              std::vector<std::string>());
    }
}


std::string DataConverter::isValid(std::string str)
{
    std::string tmp = trim(str);
    return startsWith(str, VALID);    
}

std::string DataConverter::isEmpty(std::string str)
{
    std::string tmp = trim(str);
    return startsWith(str, EMPTY);  
}

std::string DataConverter::isError(std::string str)
{
    std::string tmp = trim(str);
    return startsWith(str, ERROR);  
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
    return found!=std::string::npos && found == 0;
}