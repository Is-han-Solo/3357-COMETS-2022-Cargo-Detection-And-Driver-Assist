#include "ImageProcessing.h"
#include <iostream>

ImageProcessing::ImageProcessing()
{
	context ctx;
	auto devices = ctx.query_devices();
	auto dev = devices[0];

	if (dev.is<rs400::advanced_mode>())
   	{
       		// Get the advanced mode functionality
       		auto advanced_mode_dev = dev.as<rs400::advanced_mode>();

       		// Load and configure .json file to device
       		ifstream t("../presets/HighAccuracyPreset.json");
       		string str((istreambuf_iterator<char>(t)), istreambuf_iterator<char>());
       		advanced_mode_dev.load_json(str);
   	}

	// Starts pipeline
	pipe.start();
 
}


ImageProcessing::~ImageProcessing()
{
}

Mat ImageProcessing::getDepthImage()
{
	colorizer color_map;
	rs2::align align_to_color(RS2_STREAM_COLOR);
	frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
	data = align_to_color.process(data);
	frame depth = data.get_depth_frame().apply_filter(color_map);
	// Query frame size (width and height)
	const int w = depth.as<rs2::video_frame>().get_width();
	const int h = depth.as<rs2::video_frame>().get_height();
	// Create OpenCV matrix of size (w,h) from the colorized depth data
	Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
	// Converts color from RGB to Gray
	cvtColor(image, image, COLOR_RGB2GRAY);

	depthFrame = image;

	// Returns depth image in gray
	return image;
}

Mat ImageProcessing::getColorImage()
{
	frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
	frame color = data.get_color_frame();
	// Query frame size (width and height)
	const int w = color.as<rs2::video_frame>().get_width();
	const int h = color.as<rs2::video_frame>().get_height();
	// Create OpenCV matrix of size (w,h) from the colorized depth data
	Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
	// Converts Color from RGB to BBR
	cvtColor(image, image, COLOR_RGB2BGR);
	// Returns Color image in BGR val
	return image;
}

Mat ImageProcessing::convertToHSV(Mat image, BallColor ballColor) 
{
	// converts image from BGR to HSV
	cvtColor(image, image, COLOR_BGR2HSV);
	// thresholds HSV image for what the enum is set to. Either red or blue
	if (ballColor == BallColor::RED)
	{
		Mat image2 = image;
		inRange(image, Scalar(H_MIN_R, S_MIN_R, V_MIN_R), Scalar(H_MAX_R, S_MAX_R, V_MAX_R), image);
		inRange(image2, Scalar(H_2_MIN_R, S_MIN_R, V_MIN_R), Scalar(H_2_MAX_R, S_MAX_R, V_MAX_R), image2);

		image = image | image2;
		for (int i = 0; i < RED_EROSION; i ++)
		{
			erode(image, image, Mat());
		}

		for (int i = 0; i < RED_DILATION; i ++)
		{
			dilate(image, image, Mat());
		}
	}
	else if (ballColor == BallColor::BLUE)
	{
		inRange(image, Scalar(H_MIN_B, S_MIN_B, V_MIN_B), Scalar(H_MAX_B, S_MAX_B, V_MAX_B), image);
		for (int i = 0; i < BLUE_EROSION; i ++) // 2
		{
			erode(image, image, Mat());
		}

		for (int i = 0; i < BLUE_DILATION; i ++) // 5
		{
			dilate(image, image, Mat());
		}
	}
	// Returns filtered HSV image
	return image;
}

Mat ImageProcessing::getEdgeOfDepth(Mat image)
{
	// Mat sharp;
	// GaussianBlur(image, sharp, cv::Size(3,3), 3);
	// addWeighted(image, 1.5, sharp, -0.5, 0, sharp);
	// // Takes a depth image and draws edges around objects
	// Canny(sharp, sharp, 600, 700, 3, false);

	//cv::filter2D(image, image, -1, kernel);

	image = image > 0;

	Mat k = Mat(3,3,CV_8U);
	k.at<uchar>(0,0,0) = 0;
	k.at<uchar>(0,1,0) = 1;
	k.at<uchar>(0,2,0) = 0;
	k.at<uchar>(1,0,0) = 1;
	k.at<uchar>(1,1,0) = 1;
	k.at<uchar>(1,2,0) = 1;
	k.at<uchar>(2,0,0) = 0;
	k.at<uchar>(2,1,0) = 1;
	k.at<uchar>(2,2,0) = 0;
	cv::dilate(image, image, k, cv::Point(-1, -1), 2);

	for (int i = 0; i < 1; i++)
	{
		dilate(image, image, Mat());
	}
	// Returns this edge image
	return image;
}

Mat ImageProcessing::xorImage(Mat image, Mat edge)
{
	// Adds two images back together and returns the added image
	bitwise_and(image, edge, edge);
	// Xors the two images and then returns that
	bitwise_xor(image, edge, image);	
	// final image that is and and then xor is returned
	return image;
}

void ImageProcessing::getBallLocations(Mat image, Mat color)
{
	float cmp;

	points.clear();

	vector<Point> contourPoly;
	vector<vector<Point>> contours;
	vector<Vec4i> h;

	cvtColor(color, color, COLOR_HSV2BGR);

	findContours(image, contours, h, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0,0));
	
	for (int i = 0; i < contours.size(); i++)
	{
		//cout << "break 1" << endl;
		approxPolyDP(contours[i], contourPoly, 3, true);
		Rect_<float> boundingRectangle = boundingRect(contourPoly);
		cmp = boundingRectangle.width/boundingRectangle.height;
		if (cmp >= 0.9 && cmp <= 1.35 && boundingRectangle.area() > 1500)
		{
			double p = (double)contours[i].size();
			double a = contourArea(contours[i]);
			if (fabs(1-pow(p,2)/(4*CV_PI*a)) < 0.3)
			{
				rectangle(color, boundingRectangle.tl(), boundingRectangle.br(), Scalar(37,97,251), 5);
		
				double x = boundingRectangle.x + boundingRectangle.width/2;
				double y = boundingRectangle.y + boundingRectangle.height/2;

				points.emplace_back(std::make_pair(x,y));
			}
			//rectangle(color, boundingRectangle.tl(), boundingRectangle.br(), Scalar(37,97,251), 5);
		}

		//cout << points.size() << endl;
	}
	//imshow("final", color);
}

void ImageProcessing::convertToPolarCoords()
{
	for (int i; i < points.size(); i++)
	{
		//cout << "break 2" << endl;
		double x = points.at(i).first - 320;
		double y = points.at(i).second - 240;

		double degreesOffBall = (rsXFOV / 640) * x;

		double distanceFromBall = (rsYFOV / 480) * y;
		double differenceInHeight = camHeight - radiusOfBall;

		distanceFromBall = (distanceFromBall + camAngle) * (3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679 / 180);

		distanceFromBall = abs(differenceInHeight / tan(distanceFromBall));

		points.at(i).first = distanceFromBall;
		points.at(i).second = degreesOffBall;
		
	}
	
}

void ImageProcessing::sortPoints()
{
	std::sort(points.begin(), points.end());
}

double ImageProcessing::getClosestBallDistance()
{
	if (points.size() > 0)
	{
		return points.at(0).first;
	}
}

double ImageProcessing::getClosestBallAngle()
{
	if (points.size() > 0)
	{
		return points.at(0).second;
	}
}

void ImageProcessing::setHBlueMin(int h)
{
	H_MIN_B = h;
}

void ImageProcessing::setHBlueMax(int h)
{
	H_MAX_B = h;
}

void ImageProcessing::setSBlueMin(int s)
{
	S_MIN_B = s;
}

void ImageProcessing::setSBlueMax(int s)
{
	S_MAX_B = s;
}

void ImageProcessing::setVBlueMin(int v)
{
	V_MIN_B = v;
}

void ImageProcessing::setVBlueMax(int v)
{
	V_MAX_B = v;
}

void ImageProcessing::setHLowerRedMin(int h)
{
	H_MIN_R = h;
}

void ImageProcessing::setHLowerRedMax(int h)
{
	H_MAX_R = h;
}

void ImageProcessing::setHUpperRedMin(int h)
{
	H_2_MIN_R = h;
}

void ImageProcessing::setHUpperRedMax(int h)
{
	H_2_MAX_R = h;
}

void ImageProcessing::setSRedMin(int s)
{
	S_MIN_R = s;
}

void ImageProcessing::setSRedMax(int s)
{
	S_MAX_R = s;
}

void ImageProcessing::setVRedMin(int v)
{
	V_MIN_R = v;
}

void ImageProcessing::setVRedMax(int v)
{
	V_MAX_R = v;
}

void ImageProcessing::setCenterX(double x)
{
	centerX = x;
}

void ImageProcessing::setCenterY(double y)
{
	centerY = y;
}

void ImageProcessing::setXFOV(double fov)
{
	rsXFOV = fov;
}

void ImageProcessing::setYFOV(double fov)
{
	rsYFOV = fov;
}

void ImageProcessing::setBallRadius(double radius)
{
	radiusOfBall = radius;
}

void ImageProcessing::setCamHeight(double height)
{
	camHeight = height;
}

void ImageProcessing::setCamAngle(double angle)
{
	camAngle = angle;
}

void ImageProcessing::setRedErosion(int erosion)
{
	RED_EROSION = erosion;
}

void ImageProcessing::setRedDilation(int dilation)
{
	RED_DILATION = dilation;
}

void ImageProcessing::setBlueErosion(int erosion)
{
	BLUE_EROSION = erosion;
}

void ImageProcessing::setBlueDilation(int dilation)
{
	BLUE_DILATION = dilation;
}

