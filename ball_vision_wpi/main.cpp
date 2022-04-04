#include "ImageProcessing.h"
#include <opencv2/opencv.hpp>
//#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "cameraserver/CameraServer.h"
#include "cscore_oo.h"
#include <vector>
#include <iostream>

using namespace nt;
using namespace std;
using namespace frc;
using namespace cs;	

// 0 means blue
// 1 means red
double alliance = 0;
int hasOccured = 0;

int number = 0;

int main (int argc, char* argv[])
{	
	ImageProcessing imageProcessing;

	Mat color = imageProcessing.getColorImage();
	Mat depth;
	Mat HSV;
	Mat edge;
	Mat xorImage;

	auto inst = NetworkTableInstance::GetDefault();
	inst.StartClientTeam(3357);
	auto myNetworkTable = inst.GetTable("default");

	cs::CvSource outputStream("Realsense", VideoMode::kBGR, 160, 120, 15);
	cs::MjpegServer server("Kowalski Analysis Machine", 1182);
	server.SetSource(outputStream);

	while (true)
	{
		try 
		{
			if (hasOccured == 0)
			{
				alliance = myNetworkTable -> GetNumber("Alliance Jetson", alliance);
				imageProcessing.setHBlueMin(myNetworkTable -> GetNumber("blue h min", 90));
				imageProcessing.setHBlueMax(myNetworkTable -> GetNumber("blue h max", 98));
				imageProcessing.setSBlueMin(myNetworkTable -> GetNumber("blue s min", 210));
				imageProcessing.setSBlueMax(myNetworkTable -> GetNumber("blue s max", 255));
				imageProcessing.setVBlueMin(myNetworkTable -> GetNumber("blue v min", 120));
				imageProcessing.setVBlueMax(myNetworkTable -> GetNumber("blue v max", 255));
				imageProcessing.setHLowerRedMin(myNetworkTable -> GetNumber("red h lower min", 0));
				imageProcessing.setHLowerRedMax(myNetworkTable -> GetNumber("red h lower max", 10));
				imageProcessing.setHUpperRedMin(myNetworkTable -> GetNumber("red h upper min", 175));
				imageProcessing.setHUpperRedMax(myNetworkTable -> GetNumber("red h upper max", 180));
				imageProcessing.setSRedMin(myNetworkTable -> GetNumber("red s min", 180));
				imageProcessing.setSRedMax(myNetworkTable -> GetNumber("red s max", 255));
				imageProcessing.setVRedMin(myNetworkTable -> GetNumber("red v min", 90));
				imageProcessing.setVRedMax(myNetworkTable -> GetNumber("red v max", 255));
				imageProcessing.setCenterX(myNetworkTable -> GetNumber("realsense center x", 0));
				imageProcessing.setCenterY(myNetworkTable -> GetNumber("realsense center y", 0));
				imageProcessing.setXFOV(myNetworkTable -> GetNumber("realsense x fov", 87));
				imageProcessing.setYFOV(myNetworkTable -> GetNumber("realsense y fov", 58));
				imageProcessing.setBallRadius(myNetworkTable -> GetNumber("ball radius", 4.75));
				imageProcessing.setCamHeight(myNetworkTable -> GetNumber("realsense height", 34.5));
				imageProcessing.setCamAngle(myNetworkTable -> GetNumber("realsense angle", 26));
				imageProcessing.setRedErosion(myNetworkTable -> GetNumber("red erosion", 0));
				imageProcessing.setRedDilation(myNetworkTable -> GetNumber("red dilation", 0));
				imageProcessing.setBlueErosion(myNetworkTable -> GetNumber("blue erosion", 0));
				imageProcessing.setBlueDilation(myNetworkTable -> GetNumber("blue dilation", 0));
				hasOccured = 1;
			}
		}
		catch (...)
		{

		}
		
		color = imageProcessing.getColorImage();
		depth = imageProcessing.getDepthImage();

		if (alliance == 0)
		{
			HSV = imageProcessing.convertToHSV(color, ImageProcessing::BallColor::BLUE);	
		}
		else if (alliance == 1)
		{
			HSV = imageProcessing.convertToHSV(color, ImageProcessing::BallColor::RED);
		}

		bitwise_and(depth, HSV, depth);
		edge = imageProcessing.getEdgeOfDepth(depth);

		imageProcessing.getBallLocations(edge, color);
		imageProcessing.convertToPolarCoords();
		imageProcessing.sortPoints();

		try 
		{
			myNetworkTable -> PutNumber ("Distance To Closest Ball", imageProcessing.getClosestBallDistance());
			myNetworkTable -> PutNumber ("Angle To Closest Ball", imageProcessing.getClosestBallAngle());
		}
		catch (...)
		{

		}

		resize(color, color, Size(320, 240), INTER_LINEAR);			
		outputStream.PutFrame(color);
	
		inst.Flush();

		waitKey(10);
	}

	return 0;
}
