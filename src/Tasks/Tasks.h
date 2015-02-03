#ifndef TASK_H
#define TASK_H

#include <iostream>
#include <DetDescParams.h>
#include <ImageAcquisition.h>
#include <VisualFeatureExtraction.h>
#include <VisualFeatureEncoding.h>
#include <VisualFeatureDecoding.h>
#include <Messages/Message.h>
#include <RadioSystem/Telosb/TelosbRadioSystem.h>
#include <RadioSystem/WiFi/WiFiRadioSystem.h>
#include <RadioSystem/WiFi/ALWiFiRadioSystem.h> //ALEXIS 15/12 WIFI CLASS


using namespace std;
using namespace cv;

enum taskType {ACQUIRE_IMAGE_TASK,
	SLICE_IMAGE_TASK,
	ENCODE_SLICE_TASK,
	DECODE_SLICE_TASK,
	EXTRACT_KEYPOINTS_TASK,
	EXTRACT_FEATURES_TASK,
	ENCODE_KEYPOINTS_TASK,
	ENCODE_FEATURES_TASK,
	DECODE_KEYPOINTS_TASK,
	DECODE_FEATURES_TASK,
	SUBTRACT_BACKGROUND_TASK,
	CONVERT_COLORSPACE_TASK,
	SEND_MESSAGE_TASK,
	SEND_WIFI_MESSAGE_TASK,
	PROBE_AND_SORT_LINK_TASK,
	COMPUTE_LOADS_TASK,
	TRANSMIT_LOADS_TASK,
	//MERGE_SUBSLICES_TASK,
	SIMULATION_TASK,
};

class OffloadingManager;
class ProcessingManager;

class Task{

public:
	virtual ~Task(){};
virtual void execute() = 0;
taskType type;
bool completed;
boost::mutex task_monitor;
};


class AcquireImageTask : public Task{
private:
	ImageAcquisition *imgAcq;
	Mat image;
public:
	AcquireImageTask(ImageAcquisition *imgAcq_){
		imgAcq = imgAcq_; // the imageAcquisition object
		type = ACQUIRE_IMAGE_TASK;
		completed = false;
	}
	void execute();
	Mat getImage(){
		return image;
	}
};

class SliceImageTask : public Task{
private:
	Mat image;
	Point tl, br; // boundaries: top-left and bottom-right corners
	Mat slice;
public:
	SliceImageTask(Mat &image_, Point tl_, Point br_){
		image = image_;
		tl = tl_;
		br = br_;
		type = SLICE_IMAGE_TASK;
		completed = false;
	}
	void execute();
	Mat getSlice(){
		return slice;
	}
};

class EncodeSliceTask : public Task{
private:
	Mat image;
	int qualityFactor;
	vector<uchar> jpegBitstream;
	double encTime;
public:
	EncodeSliceTask(Mat &image_, int qualityFactor_){
		image = image_;
		qualityFactor = qualityFactor_;
		type = ENCODE_SLICE_TASK;
		completed = false;
	}
	void execute();
	vector<uchar> getJpegBitstream(){
		return jpegBitstream;
	}
	double getEncodingTime(){
		return encTime;
	}
};

class DecodeSliceTask : public Task{
private:
	vector<uchar> jpegBitstream;
	Mat decodedImage;
public:
	DecodeSliceTask(vector<uchar> &jpegBitstream_){
		jpegBitstream = jpegBitstream_;
		type = DECODE_SLICE_TASK;
		completed = false;
	}
	void execute();
	Mat getSlice(){
		return decodedImage;
	}
};

class ExtractKeypointsTask : public Task{
private:
	VisualFeatureExtraction *extractor;
	Mat image;
	vector<KeyPoint> keypoints;
	int det_thr;
	double detTime;
public:
	ExtractKeypointsTask(VisualFeatureExtraction *extractor_,Mat &image_,int det_thr_){
		extractor = extractor_;
		image = image_;
		det_thr = det_thr_;

		type = EXTRACT_KEYPOINTS_TASK;
		completed = false;
	}
	void execute();
	vector<KeyPoint> getKeypoints(){
		return keypoints;
	}
	double getDetTime(){
		return detTime;
	}
};

class ExtractFeaturesTask : public Task{
private:
	VisualFeatureExtraction *extractor;
	Mat image;
	vector<KeyPoint> keypoints;
	Mat features;
	int max_features;
	double descTime;

public:
	ExtractFeaturesTask(VisualFeatureExtraction *extractor_,
			Mat &image_,
			vector<KeyPoint> &keypoints_,
			int max_features_){
		extractor = extractor_;
		image = image_;
		keypoints = keypoints_;
		max_features = max_features_;
		type = EXTRACT_FEATURES_TASK;
		completed = false;
	}
	void execute();
	Mat getFeatures(){
		return features;
	}
	vector<KeyPoint> getKeypoints(){
		return keypoints;
	}
	double getDescTime(){
		return descTime;
	}
};

class EncodeKeypointsTask : public Task{
private:
	VisualFeatureEncoding *encoder;
	vector<KeyPoint> keypoints;
	int imWidth, imHeight;
	bool encodeAngles;
	vector<uchar> kptsBitstream;
	double kencTime;
	int method;
public:
	EncodeKeypointsTask(VisualFeatureEncoding *encoder_,
			vector<KeyPoint> &keypoints_,
			int imWidth_, int imHeight_,
			bool encodeAngles_, int method_){
		encoder = encoder_;
		keypoints = keypoints_;
		imWidth = imWidth_;
		imHeight = imHeight_;
		encodeAngles = encodeAngles_;
		type = ENCODE_KEYPOINTS_TASK;
		method = method_; // 0-> dummy, 1-> entropy coding
		completed = false;
	}
	void execute();
	vector<uchar> getKptsBitstream(){
		return kptsBitstream;
	}
	double getEncodingTime(){
		return kencTime;
	}
};

class EncodeFeaturesTask : public Task{
private:
	VisualFeatureEncoding *encoder;
	string descName;
	Mat features;
	int method;
	vector<uchar> featsBitstream;
	double fencTime;
public:
	EncodeFeaturesTask(VisualFeatureEncoding *encoder_,
			string descName_,
			Mat &features_,
			int method_){
		encoder = encoder_;
		descName = descName_;
		features = features_;
		method = method_; // 0-> dummy, 1-> entropy coding
		type = ENCODE_FEATURES_TASK;
		completed = false;
	}
	void execute();
	vector<uchar> getFeatsBitstream(){
		return featsBitstream;
	}
	double getEncodingTime(){
		return fencTime;
	}
};


class DecodeKeypointsTask : public Task{
private:
	VisualFeatureDecoding *decoder;
	vector<uchar> kptsBitstream;
	int imWidth, imHeight;
	bool decodeAngles;
	vector<KeyPoint> keypoints;
	int method;
public:
	DecodeKeypointsTask(VisualFeatureDecoding *decoder_,
			vector<uchar> &kptsBitstream_,
			int imWidth_, int imHeight_,
			bool decodeAngles_, int method_){
		decoder = decoder_;
		kptsBitstream = kptsBitstream_;
		imWidth = imWidth_;
		imHeight = imHeight_;
		decodeAngles = decodeAngles_;
		type = DECODE_KEYPOINTS_TASK;
		method = method_; // 0-> dummy, 1-> entropy coding
		completed = false;
	}
	void execute();
	vector<KeyPoint> getKeypoints(){
		return keypoints;
	}
};


class DecodeFeaturesTask : public Task{
private:
	VisualFeatureDecoding *decoder;
	string descName;
	vector<uchar> featsBitstream;
	int method;
	int numFeatures;
	Mat features;
public:
	DecodeFeaturesTask(VisualFeatureDecoding *decoder_,
			string descName_,
			vector<uchar> &featsBitstream_,
			int method_,
			int numFeatures_){
		decoder = decoder_;
		descName = descName_;
		featsBitstream = featsBitstream_;
		method = method_; // 0-> dummy, 1-> entropy coding
		numFeatures = numFeatures_;
		type = DECODE_FEATURES_TASK;
		completed = false;
	}
	void execute();
	Mat getFeatures(){
		return features;
	}
};


class SubtractBackgroundTask : public Task{
private:
	Mat sourceImage;
	Mat backgroundImage;
	Mat croppedImage;
public:
	SubtractBackgroundTask(Mat &sourceImage_,
			Mat &backgroundImage_){
		sourceImage = sourceImage_;
		backgroundImage = backgroundImage_;
		type = SUBTRACT_BACKGROUND_TASK;
		completed = false;
	}
	void execute();
	Mat getReducedImage(){
		return croppedImage;
	}
};

class ConvertColorspaceTask : public Task{
private:
	Mat sourceImage;
	int mode; // 0-> BGR2GRAY, 1-> GRAY2BGR
	Mat destImage;
public:
	ConvertColorspaceTask(Mat &sourceImage_, int mode_){
		sourceImage = sourceImage_;
		mode = mode_;
		type = CONVERT_COLORSPACE_TASK;
		completed = false;
	}
	void execute();
	Mat getConvertedImage(){
		return destImage;
	}
};

class SendMessageTask : public Task{
private:
	Message *msg_to_send;
	//this may be changed to be whatever interface
	serial_source telosb;

public:
	~SendMessageTask();
	SendMessageTask(Message *msg, serial_source radio_interface);
	void execute();

};

class SendWiFiMessageTask : public Task{
	Message *msg_to_send;

public:
	~SendWiFiMessageTask();
	SendWiFiMessageTask(Message *msg);
	void execute();
};

class ProbeAndSortLinkTask : public Task{
private:
	OffloadingManager* offloading_mng;
public:
	ProbeAndSortLinkTask(OffloadingManager* om){
		offloading_mng = om;
		type = PROBE_AND_SORT_LINK_TASK;
		completed = false;
	}
	void execute();
};

class ComputeLoadsTask : public Task{
private:
	OffloadingManager* offloading_mng;
	Mat img;
	Mat myLoad;
public:
	ComputeLoadsTask(OffloadingManager* om, Mat& image){
		offloading_mng = om;
		img = image;
		type = COMPUTE_LOADS_TASK;
		completed = false;
	}
	Mat getMyLoad(){
		return myLoad;
	}
	void execute();
};

class TransmitLoadsTask : public Task{
private:
	OffloadingManager* offloading_mng;
public:
	TransmitLoadsTask(OffloadingManager* om){
		offloading_mng = om;
		type = TRANSMIT_LOADS_TASK;
		completed = false;
	}
	void execute();
};

/*class MergeSubSlicesTask : public Task{
private:
	ProcessingManager* processing_mng;
	Mat slice;
	vector<subslice> subsliceList_;
	int subslices_iteration_;
public:
	MergeSubSlicesTask(ProcessingManager* om, int subslices_iteration, vector<int> subsliceList){
		processing_mng = om;
		subslices_iteration_ = subslices_iteration;
		subsliceList_ = subsliceList;
		type = MERGE_SUBSLICES_TASK;
		completed = false;
	}
	Mat getMySlice(){
		return slice;
	}
	void execute();
};*/

/*class ConnectToCamera : public Task{
private:
public:
	ConnectToCamera(){
		type = CONNECT_TO_CAMERA;
		completed = false;
	}
	void execute();
};*/
class SIMULATIONTask : public Task{
	Message *msg_to_send;

public:
	~SIMULATIONTask();
	SIMULATIONTask(Message *msg);
	void execute();
};

#endif
