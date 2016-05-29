#ifndef BASESTATION_SERVER_H
#define BASESTATION_SERVER_H

#include <wiringPi.h>
#include <wiringSerial.h>


namespace basestation {

    #define DEFAULT_PORT 9111
    #define REQUEST_SIZE 5
    #define SLEEP 400000

    #define DEVICE "/dev/ttyAMA0"
    #define BAUD_RATE 115200

    class BaseStationServer {
    public:
        BaseStationServer(int port = DEFAULT_PORT);
        ~BaseStationServer();

        virtual void startServer();
        bool isActive();
        void stop();

    protected:
        void activate();
        virtual std::string serialInterrogation();
        virtual void sendMeasurement(int clientFd, std::string data);


    private:
        int port;
        int serialFd;

        int socketListener;
        struct sockaddr_in *serverAddress;
        struct sockaddr_in *clientAddress;

        bool active;

        bool portIsValid(int port);

    };
}

#endif /* BASESTATION_SERVER_H */