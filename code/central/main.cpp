
#include <iostream>

#include "include/client.h"
#include "include/ekf_polling.h"

using namespace std;

int main() {

    central::Client client(10019);
    std::cout << client.askTo("192.168.1.19");
    std::cout << client.askTo("192.168.1.17");
    


    return 0;
}