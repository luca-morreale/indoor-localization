#ifndef CENTRAL_H
#define CENTRAL_H


#include <cstdlib>
#include <netdb.h>
#include <netinet/in.h>
#include <stdexcept>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>


namespace central {

    #define DEFAULT_BUFFER_SIZE 1024

    class Client {
    public:
        Client(int port, unsigned int bufferSize=DEFAULT_BUFFER_SIZE);
        ~Client();

        std::string askTo(std::string address);


    protected:
        virtual void prepareAddress(struct sockaddr_in *serv_addr, struct hostent *server);
        virtual void error(const char *msg);

    private:
        Client() { }

        int port;
        unsigned int bufferSize;

        std::string readFrom(int socketfd);
        bool hasBeenCreated(int socketfd);
        bool failedConnection(int socketfd, struct sockaddr *serv_addr);

    };

    typedef Client* ClientPtr;


} // namespace central


#endif /* CENTRAL_H */