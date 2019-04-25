#include <opencv2/imgcodecs.hpp>

#include "depth_image_annotator.h"

int main()
{
	Intrinsics intrinsics(635.246, 374.675, 923.48, 924.343, 0.0, 1280.0, 720.0, 0.0, 0.1, 3.0);
	Box bbox(643, 9, 523);
	DepthImageAnnotator dia;
	const char* fileName = "/home/eric/Dev/DepthImageAnnotator/res/depth_image.exr";
	cv::Mat depth_image = cv::imread(fileName, cv::IMREAD_UNCHANGED);
	PoseParameters params = dia.FindSolution((float*)depth_image.data, 128*128, 128, 128, true, bbox, intrinsics, 500, 5, 5);
	params.Print();
	float currentdt[256*256];
	dia.WriteImage(currentdt, 256*256, 256, 256, params, true, intrinsics);
}
