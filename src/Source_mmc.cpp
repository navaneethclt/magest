

// Include Libraries 
#include <stdio.h>
#include <NIDAQmx.h>
#include <time.h>
#include <chrono> //time
#include <windows.h> 
#include <cmath>
#include "Iir.h" //filter design
#include <iostream>
#include <fstream>
#include <Windows.h>
#include "pch.h"
#include "ArduSerial.h"
#include <math.h>   
#include "PeakFinder.h"
#include <algorithm>    // std::min_element, std::max_element
#include <vector>
#include <numeric>
#include"loopread.h"

#include <array>
#include "onnxnndetect.cpp" // Include the detector class implementation
#include "ukf.h"
#include <sstream>
#include <string>

using namespace std;
#pragma region "Auxilary functions"
//Auxilary functions
float64 maxarray(float64 arr[1000]) // find maximum value of the array
{
	for (int i = 1; i < 1000; ++i)
	{
		// Change < to > if you want to find the smallest element
		if (arr[0] < arr[i])
			arr[0] = arr[i];
	}
	//std::cout << arr[0]<<"\n";
	return arr[0];
}

// Function for calculating median
double findMedian(float64 a[], int n)
{
	// First we sort the array
	sort(a, a + n);

	// check for even case
	if (n % 2 != 0)
		return (double)a[n / 2];

	return (double)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
}
double findAvg(float64 a[], int n)
{
	double sum = 0.0;
	for (int i = 0; i < n; n++) { sum = +a[i]; }
	sum = sum / n;
	return sum;

}
void setup() //setting up the arduino connection
{
	//Serial13.begin(9600);
 //  Serial4.begin(9600);

   //Serial9.begin(9600);





	std::cout << "Connected" << std::endl;


}
void loop8(int steps) // input number of steps to travel
{

	Serial9.println(std::to_string(steps));

}
void loop4(int steps) // input number of steps to travel
{

	Serial4.println(std::to_string(steps));

}
void loopC(int steps1, int steps2) // input number of steps to travel
{

	Serial13.println(std::to_string(steps2));
	Serial4.println(std::to_string(steps1));


}

// Function to split a string based on a delimiter
std::vector<std::string> split(const std::string& s, char delimiter) {
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream(s);
	while (std::getline(tokenStream, token, delimiter)) {
		tokens.push_back(token);
	}
	return tokens;
}

float radians_to_degrees(float radians) {
	// Convert radians to degrees using the formula: angle_in_degrees = angle_in_radians * 180 / PI
	return radians * 180.0f / M_PI;
}


#pragma endregion 

#pragma region "UKF"
class MyUKF : public UKF
{

private:
	onnxnndetect detector;

public:
	MyUKF(const std::string& model_path) : detector(model_path) {}

	virtual colvec f(const colvec& x, const colvec& u) override {
		colvec xk(nStates_);
		float dt = 50 / 1000;
		xk.at(0) = x(1) * dt + x(0);
		xk.at(1) = x(1);
		xk.at(2) = x(3) * dt + x(2);
		xk.at(3) = x(3);
		//xk.print("xk=");
		return xk;
	}

	virtual colvec h(const colvec& x, const colvec& u) override {
		colvec zk(nOutputs_);
		const float offset = 403;
		float R = x(0) - offset;
		float dtheta = u(0) - x(2);
		float R2 = std::sqrt(std::pow((x(0) * std::cos(x(2)) - 25.4), 2) + std::pow((x(0) * std::sin(x(2))), 2)) - offset + 25.4;
		float theta2 = std::asin(x(0) * std::sin(x(2)) / (R2 + offset - 25.4));
		float dtheta2 = u(0) - theta2;

		std::array<float, 2> inputData = { R, radians_to_degrees(dtheta) };
		auto output = detector.detect(inputData);
		zk(0) = output[0];
		zk(1) = output[1];

		std::array<float, 2> inputData2 = { R2, radians_to_degrees(dtheta2) };
		output = detector.detect(inputData2);

		//  std::cout << u(0) << std::endl;


		zk(2) = output[2];
		zk(3) = output[3];



		return zk;
	}
};
#pragma endregion 

#pragma region "UKFII"
// Example input data
std::array<float, 2> inputData = { 1.0f, 2.0f };
std::vector<float> output;
float ref_ang = 0.0;
MyUKF myukf("your_model6_1b.onnx");
colvec x0(4);
colvec u(1);
colvec u1(4);
double Radm;
double Angm;
double Angdiff;

#pragma endregion

#pragma region "Filter Design"
//Filter Design
const float samplingrate = 20000.0;
const int order = 5;
#pragma endregion 
#pragma region "Binary Scan Variables"
//Binary Scan Variables
int stepsize = 3600; // number of steps for 1 complete revolution

#pragma endregion
#pragma region "General Variables"
int n = 1000; // number of samples per scan
float64 a1, a2; // 8 channels and their derived variables//f stands for filtered quantity
double Blmax[500000];
double Blmax2[500000];

double Bxmax[500000];
double Bxmax2[500000];

double Bymax[500000];
double Bymax2[500000];

double Bzmax[500000];
double Bzmax2[500000];

double magtime[500000];
int status = 0; //variable to control dBdt loop or 
int j = 0; // control loop count
int go; // go is the number of steps to trael
double cinput = 0;
#pragma endregion 


#pragma region "Step Training Variable"

int StepH = 0;
int StepV = 0;
int Hloop = 0;
int Vloop = 0;
int HSign = 1;
#pragma endregion

#pragma region "Angle Estimation Variables"
double Theta = 0;
#pragma endregion

#pragma region "Encoder Variables"
TaskHandle  taskHandle2 = 0; //taskhandle for the encoder channel
float ThetaAngle[500000];

#pragma endregion 
#pragma region "Callback Functions Defenition"
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else
int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData);
int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void* callbackData);
#pragma endregion
#pragma region "Clock Variables"
auto start = std::chrono::high_resolution_clock::now();
long long prev_ms = 0;
#pragma endregion
#pragma region " File"
FILE* fufilt = fopen("fileufilt.bin", "wb");//opening the file
FILE* ffilt = fopen("filefilt.bin", "wb");//opening the file
FILE* fmaxtheta = fopen("fmaxtheta_5_22_24_1_3D.csv", "wb");//opening the file



#pragma endregion

#pragma region "Loopread"
loopread loopmag;
std::vector<double> magdata = { 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0 };  // Initialize parsedVariables

#pragma endregion

int main(void)
{
#pragma region "SETUP"
	setup();//arduino setup
	loopmag.start();
#pragma endregion
#pragma region "UKFIII"
	vec  diag_elements = { 0.001, 0.001, 0.001, 0.001 };
	mat P0 = arma::diagmat(diag_elements);

	diag_elements = { 0.1, 0.01, 0.1, 0.01 };
	mat R = arma::diagmat(diag_elements);

	diag_elements = { 4, 4, 0.001, 0.001 };
	mat Q = arma::diagmat(diag_elements);
	x0 << 408 << 0 << 0 << 0;
	myukf.InitSystem(4, 4, Q, R);
	myukf.InitSystemState(x0);
	myukf.InitSystemStateCovariance(P0);
#pragma endregion
#pragma region "DAQmx config"
	int32       error = 0;
	TaskHandle  taskHandle = 0;
	char        errBuff[2048] = { '\0' };
	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("", &taskHandle));

	DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, "Dev2/ai1", "", DAQmx_Val_Diff, 0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, "Dev2/ai2", "", DAQmx_Val_Diff, 0, 10.0, DAQmx_Val_Volts, NULL));


	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", samplingrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, n));
	DAQmxErrChk(DAQmxRegisterEveryNSamplesEvent(taskHandle, DAQmx_Val_Acquired_Into_Buffer, n, 0, EveryNCallback, NULL));
	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle, 0, DoneCallback, NULL));//Registers a callback function to receive an event 
	//when a task stops due to an error or when a finite acquisition task or finite generation task completes execution. A Done event
	//does not occur when a task is stopped explicitly, such as by calling DAQmxStopTask.

	DAQmxErrChk(DAQmxCreateTask("", &taskHandle2));
	DAQmxErrChk(DAQmxCreateCIAngEncoderChan(taskHandle2, "Dev2/ctr0", "", DAQmx_Val_X4, 0, 0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Degrees, 1000, 0.0, ""));//1000 is the resolution
	DAQmxErrChk(DAQmxStartTask(taskHandle2));

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk(DAQmxStartTask(taskHandle));

	DAQmxErrChk(DAQmxSetReadRelativeTo(taskHandle, 10428));

	DAQmxErrChk(DAQmxSetReadOffset(taskHandle, -n));


	printf("Acquiring samples continuously. Press Enter to interrupt\n");
	getchar();


Error:

	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle != 0) {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
	printf("End of program, press Enter key to quit\n");
	getchar();
	return 0;
#pragma endregion


}

int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData)
{

#pragma region "General Variables"
	int32       error = 0;
	char        errBuff[2048] = { '\0' };
	int32       read = 0;
	float64     data[2000];
	float64     angledata[1]; //encoder reading
	float64     sumsqr[1000]; //sum of the square of Bx By Bz

	float64     Bzf[1000], Bz2f[1000];
	int k = 0; //k runs from 0 to 1000 aka total data count 

#pragma endregion
#pragma region "clock"
	auto elapsed = std::chrono::high_resolution_clock::now() - start;
	long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
	long long time_diff = microseconds - prev_ms;
	prev_ms = microseconds;

#pragma endregion
#pragma region "DAQ Read"
	DAQmxErrChk(DAQmxReadAnalogF64(taskHandle, float(n), 0.05, DAQmx_Val_GroupByScanNumber, data, 2 * n, &read, NULL));
	DAQmxErrChk(DAQmxReadCounterScalarF64(taskHandle2, 10.0, angledata, 0));
	auto thetaest = angledata[0];
	ThetaAngle[j] = Theta;

#pragma endregion
#pragma region "Signal Extraction"

	if (read > 0) {

		for (int i = 0; i < 2 * n; i = i + 2)


		{
			a1 = data[i];
			a2 = data[i + 1];	

			Bzf[k] = a1;
			Bz2f[k] = a2;

			k = k + 1;
		}
		fflush(stdout);
	}
	//arraymax[j] = maxarray(sumsqr);
	

	Blmax[j] = findMedian(Bzf, n);
	Blmax2[j] = findMedian(Bz2f, n);

	magdata = loopmag.getParsedVariables();
	magtime[j] = magdata[0];
	/*Bxmax[j] = (magdata[1]+28.5152)*1.05;
	Bymax[j] = 1.01*(magdata[2]-63.2283);
	Bzmax[j] = magdata[3]+89.3728;
	Bxmax2[j] = (magdata[4]+25.9419)*1.05;
	Bymax2[j] = 1.01*(magdata[5]-26.5656);
	Bzmax2[j] = magdata[6]+74.8290;*/


	Bxmax[j] = (magdata[1] + 29.6444) * 1.0396;
	Bymax[j] =  (magdata[2] - 62.6023);
	Bzmax[j] = magdata[3] + 89.3728;
	Bxmax2[j] = (magdata[4] + 26.9692) * 1.0396;
	Bymax2[j] = (magdata[5] - 26.3026);
	Bzmax2[j] = magdata[6] + 74.8290;



	double Bn1 = std::sqrt(std::pow(Bxmax[j], 2) + std::pow(Bymax[j], 2)+std::pow(Bzmax[j], 2));
	double Bn2 = std::sqrt(std::pow(Bxmax2[j], 2) + std::pow(Bymax2[j], 2) + std::pow(Bzmax2[j], 2));
	double Bn1a = -std::atan2(Bxmax[j], -Bymax[j])-ThetaAngle[j]*M_PI/180;
	double Bn2a = -std::atan2(Bxmax2[j], -Bymax2[j])- ThetaAngle[j] * M_PI / 180;

#pragma endregion

#pragma region "UKFIV"
	int backstep = 1;
	int Vmax = 5500;

	if ((j > 40) && (j % backstep == 0))
	{

		u1 << Bn1/100 << Bn1a << Bn2/100 << Bn2a;

		u << ThetaAngle[j] * M_PI / 180;


		myukf.UKalmanf(u1, u);
		colvec* x = myukf.GetCurrentState();
		colvec* x_m = myukf.GetCurrentEstimatedState();
		colvec* z = myukf.GetCurrentOutput();
		colvec* z_m = myukf.GetCurrentEstimatedOutput(); 

		Radm = x_m[0][0];
		Angm = x_m[0][2];
		std::cout << z[0][1] <<','<< z_m[0][1]<< std::endl;
		Angdiff = radians_to_degrees(Angm) - ThetaAngle[j];
		

		if (abs(Angdiff) <= 2) { cinput = Angdiff; }
		else { cinput = Angdiff * 2 / abs(Angdiff); }
		 Theta = ThetaAngle[j] + cinput;
		go = int(Theta * stepsize / 360);
		 loop8(go);
	}
#pragma endregion*/



/*#pragma region "dB/dTheta"
	int backstep = 5;
	int Vmax = 5100;//35000;//25000 //4000 for test


	if ((j > 40) && (j % backstep == 0) && (status == 0))
	{


		if (Vloop == 1)
		{
			StepV += 100;
			Vloop = 0;
			HSign = HSign * -1;
			Hloop = -5;
			if (StepV <= Vmax) loop4(StepV);

		}
		if ((Hloop < 20) && (Hloop > -1))
		{
			StepH += 5 * HSign;
			if (StepV <= Vmax)loop8(StepH);
		}
		Hloop++;

		if (Hloop == 20)
		{
			Vloop = 1;
		}

		if ((StepV > Vmax)) {Beep(500, 500);	}



		//Serial13.println("s");









			//KGain = 1;
			//Theta = ThetaAngle[j] + KGain * 0.5;
		Theta = ThetaAngle[j] + cinput;

	};


	

#pragma endregion*/
#pragma region "Initial Motor Command"

	if (j == 10)
	{
		std::cout << "Parsed Variables: ";
		for (double value : magdata) {
			std::cout << value << " ";
		}
		std::cout << std::endl;

	}
	//std::cout << Blmax2[j] * 90 + 100 << std::endl;
#pragma endregion

#pragma region "File Write"
	{
		if (StepV <= Vmax)
		{

			fprintf(fmaxtheta, "%d", j); //1
			fprintf(fmaxtheta, ",");

			fprintf(fmaxtheta, "%e", ThetaAngle[j]); //2
			fprintf(fmaxtheta, ",");

			fprintf(fmaxtheta, "%llu", microseconds);//3
			fprintf(fmaxtheta, ",");

			fprintf(fmaxtheta, "%e", Blmax[j] * 90 + 100); //4
			fprintf(fmaxtheta, ",");

			fprintf(fmaxtheta, "%e", Blmax2[j] * 90 + 100); //5
			fprintf(fmaxtheta, ",");

			fprintf(fmaxtheta, "%d", StepV);//6
			fprintf(fmaxtheta, ",");

			fprintf(fmaxtheta, "%d", StepH);//7
			fprintf(fmaxtheta, ",");

			for (double value : magdata) {
				fprintf(fmaxtheta, "%e", value);//8,9,10,11,12,13,14
				fprintf(fmaxtheta, ",");
			}
			fprintf(fmaxtheta, "%e", Radm);//15
			fprintf(fmaxtheta, ",");
			fprintf(fmaxtheta, "%e", Angm);//16 Angdiff
			fprintf(fmaxtheta, ",");
			fprintf(fmaxtheta, "%e", Angdiff);//17
			fprintf(fmaxtheta, "\n");


		}

	}

#pragma endregion
#pragma region "Print"
	cout << StepH << '\t' << StepV << '\n';
	//cout << Radm << '\t' << Angm << '\t'<< ThetaAngle[j]<<'\n';

	
#pragma endregion
#pragma region "Update"	

	j = j + 1; //controlloop count
#pragma endregion
	
#pragma region "Error"
	Error:
	if (DAQmxFailed(error)) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
		printf("DAQmx Error: %s\n", errBuff);
	}
#pragma endregion
	return 0;
}

int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void* callbackData)
{
	int32   error = 0;
	char    errBuff[2048] = { '\0' };

	// Check to see if an error stopped the task.
	DAQmxErrChk(status);

Error:
	if (DAQmxFailed(error)) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		DAQmxClearTask(taskHandle);
		printf("DAQmx Error: %s\n", errBuff);
	}
	return 0;
}

