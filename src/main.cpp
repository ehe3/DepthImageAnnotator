#include <opencv2/imgcodecs.hpp>

#include "depth_image_annotator.h"

int main()
{
	Intrinsics intrinsics(635.246, 374.675, 923.48, 924.343, 0.0, 1280.0, 720.0, 0.0, 0.1, 3.0);
	Box bbox(545, 461, 205);
	DepthImageAnnotator dia;
	const char* fileName = "/home/eric/Dev/DepthImageAnnotator/include/annoying_case.exr";
	cv::Mat depth_image = cv::imread(fileName, cv::IMREAD_UNCHANGED);
	PoseParameters params = dia.FindSolution((float*)depth_image.data, 205*205, 205, 205, true, bbox, intrinsics, 60, 1, 1);
	params.Print();
}
