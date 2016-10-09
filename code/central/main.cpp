
#include <iostream>

#include "include/client.h"

using namespace std;

int main() {

    auto client = new central::Client(10019);
    std::string payload = client->askTo("192.168.1.19");
    std::cout << payload;

    free(client);

    return 0;
}