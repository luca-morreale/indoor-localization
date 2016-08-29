#ifndef BASESTATION_DATACONVERTER_H
#define BASESTATION_DATACONVERTER_H

#include <vector>
#include <string>
#include <regex>

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>
// #include "jsoncpp/dist/json/json.h"



namespace basestation {

    typedef std::vector<std::string> VectorString;
    typedef VectorString::iterator VectorStringIterator;
    
    class DataConverter {
    public:

        static std::string convertToJson(std::string);
        static bool isValid(std::string);
        static bool isEmpty(std::string);
        static bool isError(std::string);
        

    protected:
        static std::regex reg;

        static Json::Value getEmptyJson();
        static Json::Value getErrorJson();
        static Json::Value parse(std::string str);

    private:
        static const std::string EMPTY;
        static const std::string ERROR;
        static const std::string VALID;

        static std::string trim(std::string str);
        static bool startsWith(std::string str, std::string start);


    };
}

#endif /* BASESTATION_DATACONVERTER_H */