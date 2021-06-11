#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <signal.h>

#define MMC_SUCCESS 0
#define MMC_FAILED 1

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

void* KeyHandle = 0;

//Connection Parameters
unsigned short NodeID = 1;
string g_deviceName = "EPOS4"; 
//string g_protocolStackName = "CANopen"; 
string g_protocolStackName = "MAXON SERIAL V2";
string g_interfaceName = "USB"; 
string g_portName = "USB0"; 
int g_baudrate = 1000000; 

//Motion Parameters (not SI)
//Motor 1
int v10 = 100;		//initial velocity v1(0)
int v1max = 7100;	//maximum velocity
int a1 = 500;		//acceleration
int i1 = 800;		//value of constant current during current control (mA)
int i1max = 2500;	//maximum current for motor 1
int t1 = 2000; 		//duration of current control in ms
//Motor 2
int v20 = 100;		//initial velocity v2(0)
int v2max = 10000;	//maximum velocity
int a2 = 500;		//acceleration
int i2 = 300;		//value of constant current during current control (mA)
int i2max = 1500;	//maximum current for motor 2
int t2 = 1000; 		//duration of current control in ms

int v1min = 1200;
void PrintError(string functionName, int lResult, unsigned int ErrorCode)
{
	cerr << functionName << " failed (result=" << lResult << ", errorCode=0x" << std::hex << ErrorCode << ")"<< endl;
}

void hardCapture();


int OpenDevice(unsigned int* ErrorCode)
{
	int lResult = MMC_FAILED;
	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	printf("Open device...");

	KeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, ErrorCode);

	if(KeyHandle!=0 && *ErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(KeyHandle, &lBaudrate, &lTimeout, ErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(KeyHandle, g_baudrate, lTimeout, ErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(KeyHandle, &lBaudrate, &lTimeout, ErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		KeyHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int CloseDevice(unsigned int* ErrorCode)
{
	int lResult = MMC_FAILED;

	*ErrorCode = 0;

	printf("Close device");

	if(VCS_CloseDevice(KeyHandle, ErrorCode)!=0 && *ErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}


int enableDriver(unsigned int* ErrorCode, int id)
{
	NodeID = id;
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;
	
	if(VCS_GetFaultState(KeyHandle, NodeID, &oIsFault, ErrorCode ) == 0)
	{
		PrintError("VCS_GetFaultState", lResult, *ErrorCode);
		lResult = MMC_FAILED;
		
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			if(VCS_ClearFault(KeyHandle, NodeID, ErrorCode) == 0)
			{
				PrintError("VCS_ClearFault", lResult, *ErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;
			if(VCS_GetEnableState(KeyHandle, NodeID, &oIsEnabled, ErrorCode) == 0)
			{
				PrintError("VCS_GetEnableState", lResult, *ErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{ 
				if(!oIsEnabled)
				{ 
					if(VCS_SetEnableState(KeyHandle, NodeID, ErrorCode) == 0)
					{
						PrintError("VCS_SetEnableState", lResult, *ErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}

void softCapture()
{
	NodeID = 1;
	printf("***Soft capture initiated***\n");
	int lResult;
	unsigned int ErrorCode = 0;
	short averagedCurrent = 0;

	//Velocity mode
	printf("Velocity mode.\n");
	VCS_ActivateProfileVelocityMode(KeyHandle, NodeID, &ErrorCode);
	
	//Accelerating
	printf("Accelerating...\n");
	int v = v10;
	while (v <=v1max){
		VCS_MoveWithVelocity(KeyHandle, NodeID, -v, &ErrorCode);	
		v = v + a1;
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		if (abs(averagedCurrent)>i1+500)
			break;
	}
	printf("Acceleration finished...\n");

	//Constant velocity and check current
	while (abs(averagedCurrent)<i1){
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}

	//Current mode
	printf("Switching to current mode...\n");
	VCS_ActivateCurrentMode(KeyHandle, NodeID, &ErrorCode);
	VCS_SetCurrentMust(KeyHandle, NodeID, -i1, &ErrorCode);
	for (int j=0; j<t1; j++){
		//VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	VCS_SetCurrentMust(KeyHandle, NodeID, 0, &ErrorCode);
	printf("Soft capture finished.\n");
}

void softCaptureLong()
{
	NodeID = 1;
	printf("***Soft capture long initiated***\n");
	int lResult;
	unsigned int ErrorCode = 0;
	short averagedCurrent = 0;
	int pPositionIs = -1;
	int origPos = -1;

	//Get initial position
	VCS_GetPositionIs(KeyHandle, NodeID, &pPositionIs, &ErrorCode);
	origPos = pPositionIs;
	printf("Initial position:%d\n", origPos);
	
	
	//Current mode
	VCS_ActivateCurrentMode(KeyHandle, NodeID, &ErrorCode);
	VCS_SetCurrentMust(KeyHandle, NodeID, -1800, &ErrorCode);
	printf("Velocity mode.\n");
	
	
	//Check position
	int pVelocityIsAveraged = 0;
	while (origPos-pPositionIs<600000){
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		VCS_GetPositionIs(KeyHandle, NodeID, &pPositionIs, &ErrorCode);
		VCS_GetVelocityIsAveraged(KeyHandle, NodeID, &pVelocityIsAveraged, &ErrorCode);
		//printf("Velocity:%d\n", pVelocityIsAveraged);
		//printf("Position:%d\n", origPos-pPositionIs);
		//printf("Current:%d\n",averagedCurrent);
		usleep(100);
	}
	VCS_SetCurrentMust(KeyHandle, NodeID, -700, &ErrorCode);
	sleep(1);
	for (int j=0; j<t1; j++){
		//VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	VCS_SetCurrentMust(KeyHandle, NodeID, 0, &ErrorCode);
	printf("Soft capture long finished.\n");
	hardCapture();
}

void softCaptureShort()
{
	NodeID = 1;
	printf("***Soft capture short initiated***\n");
	int lResult;
	unsigned int ErrorCode = 0;
	short averagedCurrent = 0;
	int pPositionIs = -1;
	int origPos = -1;

	//Get initial position
	VCS_GetPositionIs(KeyHandle, NodeID, &pPositionIs, &ErrorCode);
	origPos = pPositionIs;
	printf("Initial position:%d\n", origPos);
	
	
	//Current mode
	VCS_ActivateCurrentMode(KeyHandle, NodeID, &ErrorCode);
	VCS_SetCurrentMust(KeyHandle, NodeID, -1800, &ErrorCode);
	printf("Current mode.\n");
	
	
	//Check position
	int pVelocityIsAveraged = 0;
	while (origPos-pPositionIs<280000){
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		VCS_GetPositionIs(KeyHandle, NodeID, &pPositionIs, &ErrorCode);
		VCS_GetVelocityIsAveraged(KeyHandle, NodeID, &pVelocityIsAveraged, &ErrorCode);
		//printf("Velocity:%d\n", pVelocityIsAveraged);
		//printf("Position:%d\n", origPos-pPositionIs);
		//printf("Current:%d\n",averagedCurrent);
		usleep(100);
	}
	VCS_SetCurrentMust(KeyHandle, NodeID, -700, &ErrorCode);
	sleep(1);
	for (int j=0; j<t1; j++){
		//VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	VCS_SetCurrentMust(KeyHandle, NodeID, 0, &ErrorCode);
	printf("Soft capture short finished.\n");
	hardCapture();
}

void softCaptureShortSlow()
{
	NodeID = 1;
	printf("***Soft capture short slow initiated***\n");
	int lResult;
	unsigned int ErrorCode = 0;
	short averagedCurrent = 0;
	int pPositionIs = -1;
	int origPos = -1;
	
	//Accelerating
	printf("Accelerating...\n");
	int v = 100;
	while (v <=900){
		VCS_MoveWithVelocity(KeyHandle, NodeID, -v, &ErrorCode);	
		v = v + 100;
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		if (abs(averagedCurrent)>1500)
			break;
	}
	printf("Acceleration finished...\n");

	//Constant velocity and check current
	VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
	printf("Constant velocity and check current...\n");
	while (abs(averagedCurrent)<800){
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		usleep(1000);
	}

	//Current mode
	VCS_ActivateCurrentMode(KeyHandle, NodeID, &ErrorCode);
	VCS_SetCurrentMust(KeyHandle, NodeID, -700, &ErrorCode);

	for (int j=0; j<t1; j++){
		usleep(1000);
	}

	VCS_SetCurrentMust(KeyHandle, NodeID, 0, &ErrorCode);
	printf("Soft capture short slow finished.\n");
}

void softReleaseLong()
{
	NodeID = 1;
	printf("***Soft release long initiated***\n");
	int lResult;
	unsigned int ErrorCode = 0;
	short averagedCurrent = 0;

	//Velocity mode
	printf("Velocity mode.\n");
	VCS_ActivateProfileVelocityMode(KeyHandle, NodeID, &ErrorCode);
	
	//Accelerating
	printf("Accelerating...\n");
	int v = v10;
	while (v <= 1000){
		VCS_MoveWithVelocity(KeyHandle, NodeID, v, &ErrorCode);	
		v = v + 10;
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		if (abs(averagedCurrent)>2500)
			break;
	}
	printf("Acceleration finished...\n");

	//Constant velocity and check current
	while (abs(averagedCurrent)<1200){
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	
	//Current mode
	printf("Switching to current mode...\n");
	VCS_ActivateCurrentMode(KeyHandle, NodeID, &ErrorCode);
	VCS_SetCurrentMust(KeyHandle, NodeID, 1000, &ErrorCode);
	for (int j=0; j<100; j++){
		//VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	VCS_SetCurrentMust(KeyHandle, NodeID, 0, &ErrorCode);
	printf("Soft release long finished.\n");
}

void softReleaseShort()
{
	NodeID = 1;
	printf("***Soft short release short initiated***\n");
	int lResult;
	unsigned int ErrorCode = 0;
	short averagedCurrent = 0;
	int pPositionIs = -1;
	int origPos = -1;

	//Velocity mode
	printf("Velocity mode.\n");
	VCS_ActivateProfileVelocityMode(KeyHandle, NodeID, &ErrorCode);
	
	//Accelerating
	printf("Accelerating...\n");
	int v = v10;
	while (v <= 1000){
		VCS_MoveWithVelocity(KeyHandle, NodeID, v, &ErrorCode);	
		v = v + 10;
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		if (abs(averagedCurrent)>2500)
			break;
	}
	printf("Acceleration finished...\n");

	//Constant velocity and check current
	while (abs(averagedCurrent)<1200){
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	
	//Current mode
	printf("Switching to current mode...\n");
	VCS_ActivateCurrentMode(KeyHandle, NodeID, &ErrorCode);
	VCS_SetCurrentMust(KeyHandle, NodeID, 1000, &ErrorCode);
	for (int j=0; j<100; j++){
		//VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	VCS_SetCurrentMust(KeyHandle, NodeID, 0, &ErrorCode);

	//Get initial position
	VCS_GetPositionIs(KeyHandle, NodeID, &pPositionIs, &ErrorCode);
	origPos = pPositionIs;
	printf("Initial position:%d\n", origPos);

	//Velocity mode
	printf("Velocity mode.\n");
	VCS_ActivateProfileVelocityMode(KeyHandle, NodeID, &ErrorCode);
	
	//Accelerating
	printf("Accelerating...\n");
	v = v10;
	while (v <= 1000){
		VCS_MoveWithVelocity(KeyHandle, NodeID, -v, &ErrorCode);	
		v = v + 100;
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		if (abs(averagedCurrent)>1500)
			break;
	}
	printf("Acceleration finished...\n");

	//Check position
	int pVelocityIsAveraged = 0;
	VCS_GetPositionIs(KeyHandle, NodeID, &pPositionIs, &ErrorCode);
	while (origPos-pPositionIs<380000){
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		VCS_GetPositionIs(KeyHandle, NodeID, &pPositionIs, &ErrorCode);
		VCS_GetVelocityIsAveraged(KeyHandle, NodeID, &pVelocityIsAveraged, &ErrorCode);
		//printf("Velocity:%d\n", pVelocityIsAveraged);
		//printf("Position:%d\n", origPos-pPositionIs);
		//printf("Current:%d\n",averagedCurrent);
		usleep(100);
	}
	VCS_MoveWithVelocity(KeyHandle, NodeID, 0, &ErrorCode);
	printf("Soft release short finished.\n");
}

void softUnblock()
{
	NodeID = 1;
	printf("***Soft release initiated***\n");
	int lResult;
	unsigned int ErrorCode = 0;
	short averagedCurrent = 0;

	//Velocity mode
	printf("Velocity mode.\n");
	VCS_ActivateProfileVelocityMode(KeyHandle, NodeID, &ErrorCode);
	VCS_MoveWithVelocity(KeyHandle, NodeID, 800, &ErrorCode);	

	//Constant velocity and check current
	int k = 0;
	VCS_ActivateCurrentMode(KeyHandle, NodeID, &ErrorCode);
	VCS_SetCurrentMust(KeyHandle, NodeID, -500, &ErrorCode);
	usleep(100000);
	VCS_SetCurrentMust(KeyHandle, NodeID, 2500, &ErrorCode);
	while (k<100){
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		printf("Current:%d\n",averagedCurrent);
		usleep(1000);
		k++;
	}
	while (abs(averagedCurrent)<1200){
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	//Current mode
	printf("Switching to current mode...\n");
	VCS_ActivateCurrentMode(KeyHandle, NodeID, &ErrorCode);
	VCS_SetCurrentMust(KeyHandle, NodeID, 1000, &ErrorCode);
	for (int j=0; j<100; j++){
		//VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	VCS_SetCurrentMust(KeyHandle, NodeID, 0, &ErrorCode);
	printf("Soft release finished.\n");
}

void hardCapture()
{
	NodeID = 2;
	printf("***Hard capture initiated***\n");
	int lResult;
	unsigned int ErrorCode = 0;
	short averagedCurrent = 0;

	//Velocity mode
	printf("Velocity mode.\n");
	VCS_ActivateProfileVelocityMode(KeyHandle, NodeID, &ErrorCode);
	
	//Accelerating
	printf("Accelerating...\n");
	int v = v20;
	while (v <= v2max){
		VCS_MoveWithVelocity(KeyHandle, NodeID, -v, &ErrorCode);	
		v = v + a2;
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		if (abs(averagedCurrent)>i2max)
			break;
		usleep(100000);
	}
	printf("Acceleration finished...\n");

	//Constant velocity and check current
	while (abs(averagedCurrent)<i2){
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	//Current mode
	printf("Switching to current mode...\n");
	VCS_ActivateCurrentMode(KeyHandle, NodeID, &ErrorCode);
	VCS_SetCurrentMust(KeyHandle, NodeID, -i2, &ErrorCode);
	for (int j=0; j<t1; j++){
		//VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	VCS_SetCurrentMust(KeyHandle, NodeID, 0, &ErrorCode);
	printf("Hard capture finished.\n");
}

void hardRelease()
{
	NodeID = 2;
	printf("***Hard release initiated***\n");
	int lResult;
	unsigned int ErrorCode = 0;
	short averagedCurrent = 0;

	//Velocity mode
	printf("Velocity mode.\n");
	VCS_ActivateProfileVelocityMode(KeyHandle, NodeID, &ErrorCode);
	
	//Accelerating
	printf("Accelerating...\n");
	int v = v20;
	while (v <= 3000){
		//printf("v=%d\n",v);
		VCS_MoveWithVelocity(KeyHandle, NodeID, v, &ErrorCode);	
		v = v + 100;
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		if (abs(averagedCurrent)>1600)
			break;
		usleep(100000);
	}
	printf("Acceleration finished...\n");

	//Constant velocity and check current
	while (abs(averagedCurrent)<300){
		VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	//Current mode
	printf("Switching to current mode...\n");
	VCS_ActivateCurrentMode(KeyHandle, NodeID, &ErrorCode);
	VCS_SetCurrentMust(KeyHandle, NodeID, 300, &ErrorCode);
	for (int j=0; j<100; j++){
		//VCS_GetCurrentIsAveraged(KeyHandle, NodeID, &averagedCurrent, &ErrorCode);
		//printf("Current:%d\n",averagedCurrent);
		usleep(1000);
	}
	VCS_SetCurrentMust(KeyHandle, NodeID, 0, &ErrorCode);
	printf("Hard release finished.\n");
}

void enableDrivers(){
	int lResult = 0;
	unsigned int uErrorCode = 0;

	//Open connection
	if((lResult = OpenDevice(&uErrorCode))!=MMC_SUCCESS){
		PrintError("OpenDevice", lResult, uErrorCode);
	}

	//Enable drivers
	if((lResult = enableDriver(&uErrorCode,1))!=MMC_SUCCESS){
		PrintError("Enable Driver", lResult, uErrorCode);
	}
	if((lResult = enableDriver(&uErrorCode,2))!=MMC_SUCCESS){
		PrintError("Enable Driver", lResult, uErrorCode);
	}
}

void disableDrivers(){
	int lResult = 0;
	unsigned int uErrorCode = 0;

	//Disable drivers
	unsigned int Error;
	VCS_SetDisableState(KeyHandle, 1, &Error);
	VCS_SetDisableState(KeyHandle, 2, &Error);

	//Close Connection
	if((lResult = CloseDevice(&uErrorCode))!=MMC_SUCCESS){
		PrintError("CloseDevice", lResult, uErrorCode);
	}
}

void cmdCallBack(const std_msgs::String::ConstPtr& msg)
{
	if (strcmp(msg->data.c_str(),"enable") == 0)
		enableDrivers();
	if (strcmp(msg->data.c_str(),"disable") == 0)
		disableDrivers();
	if (strcmp(msg->data.c_str(),"softreleaselong") == 0)
		softReleaseLong();
	if (strcmp(msg->data.c_str(),"softreleaseshort") == 0)
		softReleaseShort();
	if (strcmp(msg->data.c_str(),"softcapture") == 0)
		softCapture();
	if (strcmp(msg->data.c_str(),"softcaptureshort") == 0)
		softCaptureShort();
	if (strcmp(msg->data.c_str(),"softcaptureshortslow") == 0)
		softCaptureShortSlow();
	if (strcmp(msg->data.c_str(),"softcapturelong") == 0)
		softCaptureLong();
	if (strcmp(msg->data.c_str(),"softunblock") == 0)
		softUnblock();
	if (strcmp(msg->data.c_str(),"hardrelease") == 0)
		hardRelease();
	if (strcmp(msg->data.c_str(),"hardcapture") == 0)
		hardCapture();
}

void test(){
	enableDrivers();
	//hardRelease();
	//softUnblock();
	//softRelease();
	//hardCapture();
	disableDrivers();
}

sig_atomic_t volatile request_shutdown = 0;
// Replacement SIGINT handler
void sigint_handler(int sig) {
	disableDrivers();
	request_shutdown = 1;
}

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "listener");
	signal(SIGINT, sigint_handler);
  	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("gripperCommand", 1, cmdCallBack);
	ros::Rate loop_rate(100);
	int lResult = MMC_FAILED;
	unsigned int uErrorCode = 0;
	unsigned int* ErrorCode;

	// enableDrivers();
	//test();	//uncomment to use the test routine
	
	//Wait forever...
	while (!request_shutdown){
		ros::spinOnce();
		loop_rate.sleep();
	}
  	
  	return 0;
}
