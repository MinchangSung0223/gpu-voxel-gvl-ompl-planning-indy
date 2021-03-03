#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include "IndyDCPConnector.h"
using namespace std;
using namespace NRMKIndy::Service::DCP;
void WaitFinish(IndyDCPConnector& connector) {
    // Wait for motion finishing
    bool fin = false;
    do {
            
        	connector.isMoveFinished(fin); // check if motion finished
    } while (!fin);
}
//Indy 초기화
int main(int argc, char* argv[]){
	IndyDCPConnector connector("192.168.0.7", ROBOT_INDY7);
        cout << "Connecting to the robot" << endl;
	connector.connect();
        cout << "OK" << endl;
	bool ready;
    connector.isRobotReady(ready);
    
	if (ready) {
        cout << "Robot is ready" << endl;

        // cout << "Go to zero position" << endl;
        // connector.moveJointZero();
        // WaitFinish(connector);
        connector.setJointBoundaryLevel(5);
        for(int  i =0 ;i<10;i++){
    		double qvec[6]  = {0,0,0,0,-9*i,0};       
       		connector.moveJointTo(qvec);
       		WaitFinish(connector);	
       		double start_values[6];
    		connector.getJointPosition(start_values);
    		cout<<start_values[0]<<","<<start_values[1]<<","<<start_values[2]<<","<<start_values[3]<<","<<start_values[4]<<","<<start_values[5]<<endl;
        }
		


        // cout << "Go to home position" << endl;
        // connector.moveJointHome();
        // WaitFinish(connector);
    }
connector.disconnect();
return -1;
}

