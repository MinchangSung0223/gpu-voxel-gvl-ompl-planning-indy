#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include "IndyDCPConnector.h"
using namespace std;
using namespace NRMKIndy::Service::DCP;

//Indy 초기화
int main(int argc, char* argv[]){
	IndyDCPConnector connector("192.168.0.7", ROBOT_INDY7);
        cout << "Connecting to the robot" << endl;
	connector.connect();
        cout << "OK" << endl;
	connector.disconnect();

return -1;
}

