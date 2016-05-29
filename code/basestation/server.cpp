#include "server.h"
#include "data_converter.h"

#include <unistd.h>
#include <runtime_error.h>

using namespace basestation;


BaseStationServer::BaseStationServer(int port = DEFAULT_PORT)
{
    if(!portIsValid(port)) {
        throw runtime_error("Port is Invalid!");
    }

    this->port = port;

    serverAddress = (sockaddr_in *) malloc(sizeof(sockaddr_in));
    clientAddress = (sockaddr_in *) malloc(sizeof(sockaddr_in));

    serialFd = serialOpen(DEVICE, BAUD_RATE);
    if(serialFd == -1) {
        throw runtime_error("Impossible to open the device!");
    }
}

void BaseStationServer::startServer()
{
    active = true;
    //create socket
    socketListener = socket(AF_INET, SOCK_STREAM, 0);
    
    if(socketListener < 0) {
        throw runtime_error("Cannot open the server socket!\n");
    }
    
    //serverAddress, clientAddress;
    bzero((char*) serverAddress, sizeof(serverAddress));

    serverAddress->sin_family = AF_INET;
    serverAddress->sin_addr.s_addr = INADDR_ANY;
    serverAddress->sin_port = htons(port);
    
    //bind socket
    if(bind(socketListener, serverAddress, sizeof(serverAddress)) < 0) {
        throw runtime_error("Can not bind the socket!\n"); 
    }
    
    listen(socketListener, REQUEST_SIZE);
    
    int len = sizeof(clientAddress);
    
    int noThread = 0;

    while(active) {
        if(noThread < 1) {
            cout << "Listening" << endl;

            //this is where client connects. svr will hang in this mode until client conn
            int connectionFd = accept(socketListener, clientAddress, &len);

            if (connectionFd < 0) {
                throw runtime_error("Cannot accept connection!\n");
                return 0;
            } else {
                cout << "Connection successful!" << endl;
            }
            
            std::string data = serialInterrogation();
            sendMeasurement(connectionFd, data);



            close(connectionFd);

        } else {
            usleep(SLEEP);
        }
    }
}

void BaseStationServer::sendMeasurement(int clientFd, std::string data)
{
    std::string tmp = DataConverter::convertToJson(data);

    int n = send(clientFd, tmp, tmp.size(), 0);
}

std::string BaseStationServer::serialInterrogation()
{
    std::string data;
    
    serialPrintf(sd, "SCANA\n");
    usleep(SLEEP);
    
    while(serialDataAvail(sd) > 0) {
        data += (char)serialGetchar(sd);
    }

    return data;
}

void BaseStationServer::activate()
{
    active = true;
}

bool BaseStationServer::isActive()
{
    return active;
}

void BaseStationServer::stop()
{
    active = false;
}

bool BaseStationServer::portIsValid(int port)
{
    return (port > 65535) || (port < 2000);
}

BaseStationServer::~BaseStationServer()
{
    stop();

    free serverAddress;
    free clientAddress;
}
