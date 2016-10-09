#ifndef BASESTATION_SERVER_H
#define BASESTATION_SERVER_H

#include <wiringPi.h>
#include <wiringSerial.h>

#include <arpa/inet.h>
#include <iostream>
#include <netinet/in.h>
#include <netinet/ip.h> 
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

#include "serial_reader.h"


namespace basestation {

    #define DEFAULT_PORT 9111
    #define QUEUE_SIZE 5 

    class BaseStationServer {
    public:
        BaseStationServer(int port=DEFAULT_PORT);
        ~BaseStationServer();

        virtual void startServer();
        bool isActive();
        void stop();

    protected:
        virtual void activate();
        virtual int getClient();
        virtual int createSocket();
        virtual void createServer();
        virtual void answerRequest();
        virtual void setServerOptions();

        virtual void sendMeasurement(int clientFd, const std::string data);

        virtual void logClientConnection(int clientFd);


    private:
        int port;
        SerialReader *reader;

        int socketListener;
        struct sockaddr_in *serverAddress;
        struct sockaddr_in *clientAddress;

        bool active;

        bool portIsValid(int port);
        bool isBindSuccessful();
        void checkConnection(int clientFd);
        bool connectionSuccessful(int fd);
        void getSocketAddr(struct sockaddr_in &addr, int fd);

    };
}

#endif /* BASESTATION_SERVER_H */