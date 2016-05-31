#ifndef BASESTATION_DATACONVERTER_H
#define BASESTATION_DATACONVERTER_H



namespace basestation {

    
    class DataConverter {
    public:

        static std::string convertToJson(std::string);
        static bool isValid(std::string);
        static bool isEmpty(std::string);
        static bool isError(std::string);
        

    protected:
        


    private:
        static const std::string EMPTY;
        static const std::string ERROR;
        static const std::string VALID;

        static std::string trim(std::string str);
        static bool startsWith(std::string str, std::string start);


    };
}

#endif /* BASESTATION_DATACONVERTER_H */