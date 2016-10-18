#include "include/client.h"
#include <iostream>


using namespace central;

Client::Client(int port, unsigned int bufferSize)
{
    this->port = port;
    this->bufferSize = bufferSize;
}

std::string Client::askTo(std::string address)
{
    struct sockaddr_in serv_addr;
    struct hostent *server;

    server = gethostbyname(address.c_str());


    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    
    if (!hasBeenCreated(sockfd)) {
        error("ERROR opening socket");
        //throw error
    }

    prepareAddress(&serv_addr, server);

    if (failedConnection(sockfd, (struct sockaddr *) &serv_addr)) {
        error("ERROR connecting");
        //throw error
    }
    std::cout << "reading\n";
    std::string payload = this->readFrom(sockfd);
    std::cout << "error reading\n";
    close(sockfd);

    return payload;
}

std::string Client::readFrom(int socketfd)
{
    std::string buffer(this->bufferSize, 0);
    read(socketfd, &buffer[0], this->bufferSize);

    return buffer;
}

void Client::prepareAddress(struct sockaddr_in *serv_addr, struct hostent *server)
{
    bzero((char *) serv_addr, sizeof(*serv_addr));
    serv_addr->sin_family = AF_INET;

    bcopy((char *) server->h_addr, (char *) &serv_addr->sin_addr.s_addr, server->h_length);
    serv_addr->sin_port = htons(this->port);
}


bool Client::hasBeenCreated(int socketfd)
{
    return socketfd >= 0;
}


bool Client::failedConnection(int socketfd, struct sockaddr *serv_addr)
{
    return connect(socketfd, serv_addr, sizeof(*serv_addr)) < 0;
}

void Client::error(const char *msg)
{
    perror(msg);
    exit(0);
}


Client::~Client()
{ /*   */ }