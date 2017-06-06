#ifndef _IN_ARI_GESTURE_
#define _IN_ARI_GESTURE_

#include <opencv2/opencv.hpp>

//using namespace cv;
using namespace std;

//const int --can be modefied
const int cube_size = 200;
const int trajFrameNum = 6;
const int ntimes = 4;


//background updating
const float AvgThreshold = 25;//background update threshold
const int calStep = 2;//calculatiion step
const int calLength = 15;//updating frame num
const int obstrArea = 1000;//blob size threshhold
const int windowStep = 30 * 60;//updating time

//to be used
struct InAirGestureInfo{
	bool graspstate;
	bool openstate;
	int wavestate;
};

//in air hand state
//1.hand wave
enum HandWave{
	NOWAVE,
	WAVERIGHT,
	WAVELEFT
	//WAVEUP
	//WAVEDOWN
};
//2.hand grasp
enum HandGrasp{
	GRASPFAITTURE,
	GRASPSUCCESS
};
//3.hand open
enum HandOpen{
	OPENFAITURE,
	OPENSUCCESS
};

struct ConvexityDefect
{
	cv::Point start;
	cv::Point end;
	cv::Point depth_point;
	float depth;
};

#endif