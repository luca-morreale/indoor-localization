#include "basestation_server.h"
#include "data_converter.h"

#include <stdexcept>
#include <unistd.h>
#include <iostream>

using namespace basestation;


BaseStationServer::BaseStationServer(int port)
{
    if(!portIsValid(port)) {
        throw std::runtime_error("Port is Invalid!");
    }

    this->port = port;

    serverAddress = (sockaddr_in *) malloc(sizeof(sockaddr_in));
    clientAddress = (sockaddr_in *) malloc(sizeof(sockaddr_in));

    reader = new SerialReader();
}

bool BaseStationServer::portIsValid(int port)
{
    return (port < 65535) && (port > 2000);
}

void BaseStationServer::startServer()
{
    createServer();
    
    if(!isBindSuccessful()) {
        throw std::runtime_error("Can not bind the socket!\n"); 
    }
    
    listen(socketListener, QUEUE_SIZE);

    while(active) {
        std::cout << "Listening" << std::endl;

        answerRequest();
    }
}

int BaseStationServer::createSocket()
{
    return socket(AF_INET, SOCK_STREAM, 0);
}

void BaseStationServer::createServer()
{
    activate();
    socketListener = createSocket();
    
    if(socketListener < 0) {
        throw std::runtime_error("Cannot open the server socket!\n");
    }
    //copy zeros into string: serverAddress, clientAddress;
    bzero((char*) serverAddress, sizeof(serverAddress));
    setServerOptions();
    
}

void BaseStationServer::setServerOptions()
{
    serverAddress->sin_family = AF_INET;
    serverAddress->sin_addr.s_addr = INADDR_ANY;
    serverAddress->sin_port = htons(port);
}

void BaseStationServer::answerRequest()
{
    int clientFd = getClient();

    std::string json = reader->getJsonScan();
    sendMeasurement(clientFd, json);

    close(clientFd);
}

int BaseStationServer::getClient()
{
    socklen_t len = sizeof(clientAddress);
    //this is where client connects. server will hang in this mode until client conn
    int clientFd = accept(socketListener, (sockaddr*) clientAddress, &len);
    checkConnection(clientFd);

    return clientFd;
}

void BaseStationServer::checkConnection(int clientFd)
{
    if (!connectionSuccessful(clientFd)) {
        throw std::runtime_error("Cannot accept connection!\n");
    } else {
        logClientConnection(clientFd);
    }
}

void BaseStationServer::getSocketAddr(struct sockaddr_in &addr, int fd)
{
    socklen_t addr_size = sizeof(struct sockaddr_in);
    getpeername(fd, (struct sockaddr *)&addr, &addr_size);
}

bool BaseStationServer::isBindSuccessful()
{
    return bind(socketListener, (const sockaddr*) serverAddress, sizeof(*serverAddress)) >= 0;
}

bool BaseStationServer::connectionSuccessful(int fd)
{
    return fd >= 0;
}

void BaseStationServer::sendMeasurement(int clientFd, std::string json)
{
    const void *out = json.c_str();
    int n = send(clientFd, out, json.size(), 0);
}

bool BaseStationServer::isActive()
{
    return active;
}

void BaseStationServer::activate()
{
    active = true;
}

void BaseStationServer::stop()
{
    active = false;
}

void BaseStationServer::logClientConnection(int clientFd)
{
    struct sockaddr_in addr;
    getSocketAddr(addr, clientFd);
    std::cout << "Connection successful! " << inet_ntoa(addr.sin_addr) << std::endl;
}

BaseStationServer::~BaseStationServer()
{
    stop();

    delete(reader);
    std::free(serverAddress);
    std::free(clientAddress);
}
