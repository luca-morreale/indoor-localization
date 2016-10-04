
#include "basestation_server.h"


using namespace std;


int main()
{
	basestation::BaseStationServer server(10019);
	
	server.startServer();

	return 0;
}