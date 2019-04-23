#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <opencv2/core.hpp>
#include "opencv2/calib3d/calib3d.hpp"

using namespace std;
using namespace cv;

int main() {
    ifstream ifs("profile.json");
    Json::Reader reader2;
    Json::Value obj;
    reader2.parse(ifs, obj);     // Reader can also read strings
    cout << "Last name: " << obj["lastname"].asString() << endl;
    cout << "First name: " << obj["firstname"].asString() << endl;

    //Read camera json file
    ifstream inputfile("0.json");
    Json::Reader reader;
    Json::Value object;
    reader.parse(inputfile, object);
	Mat kk(3, 3, cv::DataType<float>::type, Scalar(0));
	Mat rotm(3, 3, cv::DataType<float>::type, Scalar(1));
	Mat Rt(3, 4, cv::DataType<float>::type, Scalar(1));
	Mat tvec(3, 1, cv::DataType<float>::type, Scalar(1));
	Mat quat(4, 1, cv::DataType<float>::type, Scalar(1));

	kk.at<float>(0, 0) = object["camera"]["focalLength"]["x"].asFloat();
	kk.at<float>(0, 1) = 0;
	kk.at<float>(0, 2) = object["camera"]["principalPoint"]["x"].asFloat();
	kk.at<float>(1, 0) = 0;
	kk.at<float>(1, 1) = object["camera"]["focalLength"]["y"].asFloat();
	kk.at<float>(1, 2) = object["camera"]["principalPoint"]["y"].asFloat();
	kk.at<float>(2, 0) = 0;
	kk.at<float>(2, 1) = 0;
	kk.at<float>(2, 2) = 1;

	//push kk

	float qw = quat.at<float>(0, 0) = object["camera"]["rotation"]["w"].asFloat();
	float qx = quat.at<float>(0, 1) = object["camera"]["rotation"]["x"].asFloat();
	float qy = quat.at<float>(0, 2) = object["camera"]["rotation"]["y"].asFloat();
	float qz = quat.at<float>(0, 3) = object["camera"]["rotation"]["z"].asFloat();

	//push quat

	Mat rvec(3, 1, cv::DataType<float>::type, Scalar(1));
	float angle = 2 * acos(qw);
	rvec.at<float>(0, 0) = (qx / sqrt(1 - qw * qw)) * angle;
	rvec.at<float>(1, 0) = (qy / sqrt(1 - qw * qw)) * angle;
	rvec.at<float>(2, 0) = (qz / sqrt(1 - qw * qw)) * angle;
	Rodrigues(rvec, rotm);

	rotm = rotm.t();

	//push rotm

	tvec.at<float>(0,0) = object["camera"]["position"]["x"].asFloat();
	tvec.at<float>(1,0) = object["camera"]["position"]["y"].asFloat();
	tvec.at<float>(2,0) = object["camera"]["position"]["z"].asFloat();

	//push tvec to Cs

	tvec = -rotm * tvec;

	//push tvec to ts

	Rt.at<float>(0, 0) = rotm.at<float>(0, 0);
	Rt.at<float>(0, 1) = rotm.at<float>(0, 1);
	Rt.at<float>(0, 2) = rotm.at<float>(0, 2);
	Rt.at<float>(1, 0) = rotm.at<float>(1, 0);
	Rt.at<float>(1, 1) = rotm.at<float>(1, 1);
	Rt.at<float>(1, 2) = rotm.at<float>(1, 2);
	Rt.at<float>(2, 0) = rotm.at<float>(2, 0);
	Rt.at<float>(2, 1) = rotm.at<float>(2, 1);
	Rt.at<float>(2, 2) = rotm.at<float>(2, 2);
	Rt.at<float>(0, 3) = tvec.at<float>(0, 0);
	Rt.at<float>(1, 3) = tvec.at<float>(1, 0);
	Rt.at<float>(2, 3) = tvec.at<float>(2, 0);

	//push Rt

	Mat Ptemp = kk * Rt;

	cout << "camera intrinsics  : "<<endl;
	cout<< kk << endl;
	cout << "camera position  : "<<endl;
	cout << tvec << endl;
	cout << "camera rotation  : "<<endl;
	cout<< rotm << endl;
	cout << "camera R|t  : "<<endl;
	cout << Rt << endl;
	cout << "camera P  : "<<endl;
	cout << Ptemp << endl;



	//push Ptemp to Ps

//    cout << "camera position x : " << object["camera"]["position"]["x"] << endl;


//    cout << "frameID  : " << object["frameId"] << endl;

    return 1;
}
