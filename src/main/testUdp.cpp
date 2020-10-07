
#include "ClientUDP.h"

int main() {

    ClientUDP clientUdp("172.24.1.148", 1234);
    clientUdp.sendMessage("Test working");

    return 0;
}