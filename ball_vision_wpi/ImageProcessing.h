#pragma once

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
#include <math.h>

using namespace std;
using namespace cv;
using namespace rs2;

class ImageProcessing
{

public:

	// BALL COLOR ENUM

	enum BallColor 
	{
		RED, 
		BLUE
	};

	// CONSTRUCTOR

	ImageProcessing();

	// DESTRUCTOR

	~ImageProcessing();

	// RETURN TYPE MAT FUNCTIONS

	Mat getDepthImage();
	Mat getColorImage();
	Mat convertToHSV(Mat image, BallColor ballColor);
	Mat getEdgeOfDepth(Mat image);
	Mat xorImage(Mat image, Mat edge);
	void getBallLocations(Mat image, Mat color);
	void convertToPolarCoords();
	void sortPoints();
	double getClosestBallDistance();
	double getClosestBallAngle();
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
setters / getters for setting params
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

	void setHBlueMin(int h);
	void setHBlueMax(int h);
	void setSBlueMin(int s); 
	void setSBlueMax(int s);
	void setVBlueMin(int v);
	void setVBlueMax(int v);
	void setHLowerRedMin(int h);
	void setHLowerRedMax(int h);
	void setHUpperRedMin(int h);
	void setHUpperRedMax(int h);
	void setSRedMin(int s);
	void setSRedMax(int s);
	void setVRedMin(int s);
	void setVRedMax(int s);
	void setCenterX(double x);
	void setCenterY(double y);
	void setXFOV(double fov);
	void setYFOV(double fov);
	void setBallRadius(double radius);
	void setCamHeight(double height);
	void setCamAngle(double angle);
	void setRedErosion(int erosion);
	void setRedDilation(int dilation);
	void setBlueErosion(int erosion);
	void setBlueDilation(int dilation);
	
private:

	// PIPELINE FOR ALL PROCESSING
	
	pipeline pipe;

	// HSV FILTERING VALS

	// H filter for blue
	int H_MIN_B = 90;
	int H_MAX_B = 98;

	// S filter for blue
	int S_MIN_B = 210;
	int S_MAX_B = 255;
	
	// V filter for blue
	int V_MIN_B = 120;
	int V_MAX_B = 255;

	//  H filter for red
	int H_MIN_R = 0;
	int H_MAX_R = 10;

	// S filter for red
	int S_MIN_R = 180;
	int S_MAX_R = 255;

	// V filter for red
	int V_MIN_R = 90;
	int V_MAX_R = 255;

	// SECONDARY H FILTER FOR RED

	int H_2_MIN_R = 175;
	int H_2_MAX_R = 180;

	// EROSION / DILATION OF HSV VALS
	
	int RED_EROSION;
	int RED_DILATION;

	int BLUE_EROSION;
	int BLUE_DILATION;

	// KERNEL 

	const Mat kernel = (Mat_<float>(3, 3)<<1, 1, 1, 1, -9, 1, 1, 1, 1);

	// const Mat kernel = (Mat_<float>(3, 3)<<0, 1, 0, 1, -8, 0, 0, 1, 0);

	vector<pair<double, double>> points;

	double centerX = 0;
	double centerY = 0;

	double rsXFOV = 87;
	double rsYFOV = 58;

	double radiusOfBall = 4.75;
	double camHeight = 34.5; // TODO gotta change this

	double camAngle = 22.4;

	Mat stackedImage;

	Mat depthFrame;
	
};
