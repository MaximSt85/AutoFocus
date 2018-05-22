#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp>  // Gaussian Blur
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <string>
#include <sstream>
#include <unistd.h>
#include <ctime>

#include <tango.h>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

#include <vector>

#include <thread>

using namespace curlpp::options;
using namespace cv;
using namespace std;


class Camera{
public:
	void switchStartFrameTriggerMode(int mode);
	int fixedRate = 3;
	int triggerMode = 1;
	int freerunMode = 0;
private:
	
};

void Camera::switchStartFrameTriggerMode(int mode){
	try {
		curlpp::Cleanup myCleanup;
		curlpp::Easy myRequest;
		if (mode == 0) {
			myRequest.setOpt<Url>("http://192.168.39.1:8082/?action=command&id=13&dest=0&group=42&value=0&plugin=0");
		}
		else if (mode == 1) {
			myRequest.setOpt<Url>("http://192.168.39.1:8082/?action=command&id=13&dest=0&group=42&value=1&plugin=0");
		}
		else if (mode == 3) {
			myRequest.setOpt<Url>("http://192.168.39.1:8082/?action=command&id=13&dest=0&group=42&value=3&plugin=0");
		}
		else {return;}
		myRequest.perform();
	}
	catch(curlpp::RuntimeError & e)	{
		cout << e.what() << endl;
	}
	catch(curlpp::LogicError & e) {
		cout << e.what() << endl;
	}
}


class findFocus{
public:
	findFocus(vector<Mat> imageArray1, vector<double> positionsPiLC1);
	double findPosition();
private:
	vector<double> positionsPiLC;
	vector<Mat> imageArray;
};

findFocus::findFocus(vector<Mat> imageArray1, vector<double> positionsPiLC1) {
	positionsPiLC = positionsPiLC1;
	imageArray = imageArray1;
}

double findFocus::findPosition() {
	double position;
	vector<double> sharpnesses;
	//int arraySize = positionsPiLC.size();
	int arraySize = imageArray.size();
	Mat src, gray, dst;
	Scalar mu, sigma;
	for (int i=0;i<arraySize;i++) {
		src = imageArray[i];
		cvtColor(src, gray, CV_RGB2GRAY);
		Laplacian(gray, dst, CV_64F);
		meanStdDev(dst, mu, sigma);
		double focusMeasure = sigma.val[0] * sigma.val[0];
		sharpnesses.push_back(focusMeasure);
	}
	vector<double>::iterator index = max_element(sharpnesses.begin(), sharpnesses.end());
	int indexMaxSharpness;
	indexMaxSharpness = distance(sharpnesses.begin(), index);
	//cout << "indexMaxSharpness1: " << indexMaxSharpness1 << endl;
	//auto index = find(sharpnesses.begin(), sharpnesses.end(), maxSharpness);
	return indexMaxSharpness*10;
}

class pilc{
public:
	pilc(string path_to_device);

	void set_arm(int on_off);
	void set_nbTriggers(int nbTriggers_to_set);
	void set_positionTriggerStart(double positionTriggerStart_to_set);
	void set_positionTriggerStepSize(double positionTriggerStepSize_to_set);
	void set_timeTriggerStepSize(double timeTriggerStepSize_to_set);
	vector<double> get_position1Data();
	
private:
	Tango::DeviceProxy *pilc_device;	
	
	Tango::DeviceAttribute nbTriggers_attribute;
	Tango::DeviceAttribute positionTriggerStart_attribute;
	Tango::DeviceAttribute arm_attribute;
	Tango::DeviceAttribute timeTriggerStepSize_attribute;
	Tango::DeviceAttribute positionTriggerStepSize_attribute;
	Tango::DeviceAttribute position1Data_attribute;

	double positionTriggerStart;
	long arm;
	int nbTriggers;
	double timeTriggerStepSize;
	double positionTriggerStepSize;
	double calibrateEncoder1;
	vector<double> position1Data;

	string nbTriggers_name = "NbTriggers";
	string positionTriggerStart_name = "PositionTriggerStart";
	string arm_name = "Arm";
	string timeTriggerStepSize_name = "TimeTriggerStepSize";
	string positionTriggerStepSize_name = "PositionTriggerStepSize";
	string calibrateEncoder1_name = "CalibrateEncoder1";
	string position1Data_name = "Position1Data";
};

pilc::pilc(string path_to_device) {
	try {
		pilc_device = new Tango::DeviceProxy(path_to_device);
		arm_attribute = pilc_device->read_attribute(arm_name);
		nbTriggers_attribute = pilc_device->read_attribute(nbTriggers_name);
		positionTriggerStart_attribute = pilc_device->read_attribute(positionTriggerStart_name);
		positionTriggerStepSize_attribute = pilc_device->read_attribute(positionTriggerStepSize_name);
		timeTriggerStepSize_attribute = pilc_device->read_attribute(timeTriggerStepSize_name);
	}
	catch (Tango::DevFailed &e) {
		Tango::Except::print_exception(e);
    	}
}

void pilc::set_arm(int on_off) {
	if (on_off == 1) {
		try {
			//arm_attribute = pilc_device->read_attribute(arm_name);
			//arm_attribute >> arm;
			Tango::DevVarLongArray *in = new Tango::DevVarLongArray();
			in->length(2);
			(*in)[0] = 1;
			(*in)[1] = 1;
			arm_attribute << in;
			pilc_device->write_attribute(arm_attribute);
  		}
		catch (Tango::DevFailed &e) {
			Tango::Except::print_exception(e);
	    	}
	}
	else {
		try {
			//arm_attribute = pilc_device->read_attribute(arm_name);
			//arm_attribute >> arm;
			Tango::DevVarLongArray *in = new Tango::DevVarLongArray();
			in->length(2);
			(*in)[0] = 1;
			(*in)[1] = 0;
			arm_attribute << in;
			pilc_device->write_attribute(arm_attribute);
  		}
		catch (Tango::DevFailed &e) {
			Tango::Except::print_exception(e);
	    	}
	}
}

void pilc::set_nbTriggers(int nbTriggers_to_set) {
	try {
		//nbTriggers_attribute = pilc_device->read_attribute(nbTriggers_name);
		//nbTriggers_attribute >> nbTriggers;
		nbTriggers = nbTriggers_to_set;
		nbTriggers_attribute << nbTriggers;
		pilc_device->write_attribute(nbTriggers_attribute);
	}
	catch (Tango::DevFailed &e) {
		Tango::Except::print_exception(e);
	}
}

void pilc::set_positionTriggerStart(double positionTriggerStart_to_set) {
	try {
		//positionTriggerStart_attribute = pilc_device->read_attribute(positionTriggerStart_name);
		//positionTriggerStart_attribute >> positionTriggerStart;
		positionTriggerStart = positionTriggerStart_to_set;
		positionTriggerStart_attribute << positionTriggerStart;
		pilc_device->write_attribute(positionTriggerStart_attribute);
	}
	catch (Tango::DevFailed &e) {
		Tango::Except::print_exception(e);
	}
}

void pilc::set_positionTriggerStepSize(double positionTriggerStepSize_to_set) {
	try {
		//positionTriggerStepSize_attribute = pilc_device->read_attribute(positionTriggerStepSize_name);
		//positionTriggerStepSize_attribute >> positionTriggerStepSize;
		positionTriggerStepSize = positionTriggerStepSize_to_set;
		positionTriggerStepSize_attribute << positionTriggerStepSize;
		pilc_device->write_attribute(positionTriggerStepSize_attribute);
	}
	catch (Tango::DevFailed &e) {
		Tango::Except::print_exception(e);
	}
}

void pilc::set_timeTriggerStepSize(double timeTriggerStepSize_to_set) {
	try {
		//timeTriggerStepSize_attribute = pilc_device->read_attribute(timeTriggerStepSize_name);
		//timeTriggerStepSize_attribute >> timeTriggerStepSize;
		timeTriggerStepSize = timeTriggerStepSize_to_set;
		timeTriggerStepSize_attribute << timeTriggerStepSize;
		pilc_device->write_attribute(timeTriggerStepSize_attribute);
	}
	catch (Tango::DevFailed &e) {
		Tango::Except::print_exception(e);
	}
}

vector<double> pilc::get_position1Data() {
	try {
		position1Data_attribute = pilc_device->read_attribute(position1Data_name);
		position1Data_attribute >> position1Data;
		return position1Data;
	}
	catch (Tango::DevFailed &e) {
		Tango::Except::print_exception(e);
	}
}

class y_axis{
public:
	y_axis(string path_to_device);
	
	void set_position(double position_to_set);
	int get_state();
	void set_velocity(double velocity_to_set);
private:
	Tango::DeviceProxy *y_axis_device;	
	
	Tango::DeviceAttribute position_attribute;
	Tango::DeviceAttribute velocity_attribute;
	double position;
	string position_name = "Position";
	string velocity_name = "Velocity";
	int state;
};

y_axis::y_axis(string path_to_device) {
	y_axis_device = new Tango::DeviceProxy(path_to_device);
	position_attribute = y_axis_device->read_attribute(position_name);
	velocity_attribute = y_axis_device->read_attribute(velocity_name);
}

void y_axis::set_position(double position_to_set) {
	try {
		Tango::DevVarDoubleArray *in = new Tango::DevVarDoubleArray();
		in->length(1);
		(*in)[0] = position_to_set;
		position_attribute << in;
		y_axis_device->write_attribute(position_attribute);
  	}
	catch (Tango::DevFailed &e) {
		Tango::Except::print_exception(e);
	}
	catch (CORBA::BAD_PARAM &e) {
		Tango::Except::print_exception(e);
	}
}

int y_axis::get_state() {
	try {
		state = y_axis_device->state();
		return state;
  	}
	catch (Tango::DevFailed &e) {
		Tango::Except::print_exception(e);
	}
}

void y_axis::set_velocity(double velocity_to_set) {
	try {
		Tango::DevVarDoubleArray *in = new Tango::DevVarDoubleArray();
		in->length(1);
		(*in)[0] = velocity_to_set;
		velocity_attribute << in;
		y_axis_device->write_attribute(velocity_attribute);
  	}
	catch (Tango::DevFailed &e) {
		Tango::Except::print_exception(e);
	}
	catch (CORBA::BAD_PARAM &e) {
		Tango::Except::print_exception(e);
	}
}

void takeImages(vector<Mat>& imageArray, VideoCapture& cap, int nbTriggers) {
	int counter = 0;
	int counter1 = 0;
	nbTriggers -= 1;
	while(true) {
		auto begin = chrono::high_resolution_clock::now();
		Mat img;
		cap >> img;
		auto end = chrono::high_resolution_clock::now();
		float elapsed_secs = chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count()/1000000000;
		cout << "elapsed_secs: " << elapsed_secs << endl;
		counter1++;
		cout << "counter1: " << counter1 << endl;
		if (elapsed_secs > 0.030) {
			imageArray.push_back(img);
			break;
		}
	}
	//namedWindow("Video", WINDOW_AUTOSIZE);
	counter++;
	cout << "taking pictures when triggering..." << endl;
	for (;;) {
		Mat img1;
		cap >> img1;
		//imshow("Video", img1);
		//int key = waitKey(1);
		//cv::Mat dst;
		//cv::flip(img1, dst, 0);
		imageArray.push_back(img1);
		counter++;
		cout << "counter is: " << counter << endl;
		ostringstream ss;
		ss << counter;
		string filename;
		filename = ss.str() + ".jpg";
		imwrite(filename, img1);
		if (counter == nbTriggers) {break;}
	}
}

int main(int argc, char** argv )
{
	//float conversionFactor = 1.81;
	
	vector<double> position1Data;
	vector<Mat> imageArray;
	
	int nbTriggers;
	double velocity1_y_axe = 3500;
	double velocity2_y_axe = 100;
	double start_position_y_axe = -300;
	double end_position_y_axe = 300;
	double positionTriggerStart = -300;
	double timeTriggerStepSize = 0.1;
	nbTriggers = (end_position_y_axe - start_position_y_axe) / (velocity2_y_axe * timeTriggerStepSize);

	pilc myPilc("haspp11mexp5:10000/p11/pilc/pilc-trigger-generator");
	y_axis myY_axis("haspp11mexp5:10000/p11/motor/galil.06");

	myY_axis.set_velocity(velocity1_y_axe);
	myY_axis.set_position(start_position_y_axe);
	while (myY_axis.get_state() == Tango::MOVING) {
		sleep(0.1);
	}
	myPilc.set_nbTriggers(nbTriggers);
	myPilc.set_positionTriggerStart(positionTriggerStart);
	myPilc.set_timeTriggerStepSize(timeTriggerStepSize);
	cout << "########## all parameters setted ##########" << endl;

	Camera myCamera;
	myCamera.switchStartFrameTriggerMode(myCamera.freerunMode);
	//myCamera.switchStartFrameTriggerMode(myCamera.fixedRate);
	VideoCapture cap("http://192.168.39.1:8082/?action=stream/frame.mjpg");
	std::thread takeImagesThread (takeImages, ref(imageArray), ref(cap), nbTriggers);
	myCamera.switchStartFrameTriggerMode(myCamera.triggerMode);
	sleep(2);
	cout << "########## captured video ##########" << endl;

	myPilc.set_arm(1);
	myY_axis.set_velocity(velocity2_y_axe);
	myY_axis.set_position(end_position_y_axe);
	takeImagesThread.join();
	//position1Data = myPilc.get_position1Data();
	//cout << "size of position1Data: " << position1Data.size() << endl;
	cout << "size of imageArray: " << imageArray.size() << endl;
	cout << "########## finished scan ##########" << endl;
	
	findFocus myFocus(imageArray, position1Data);
	double position = myFocus.findPosition();
	cout << "########## found position to move ########## " << position << endl;

	myCamera.switchStartFrameTriggerMode(myCamera.freerunMode);

	myY_axis.set_velocity(velocity1_y_axe);
	while (myY_axis.get_state() == Tango::MOVING) {
		sleep(0.1);
	}
	//myY_axis.set_position(start_position_y_axe);
	myY_axis.set_position(start_position_y_axe + position - 10);
	while (myY_axis.get_state() == Tango::MOVING) {
		sleep(0.1);
	}

	return 0;
}
