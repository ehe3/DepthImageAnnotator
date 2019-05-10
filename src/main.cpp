#include <opencv2/imgcodecs.hpp>

#include "depth_image_annotator.h"

int main()
{
	Intrinsics intrinsics(644.489, 358.173, 922.249, 922.439, 0.0, 1280.0, 720.0, 0.0, 0.1, 3.0);
	Box bbox(563, 338, 164);
	DepthImageAnnotator dia;
	const char* fileName = "/home/eric/Dev/DepthImageAnnotator/include/annoying_case.exr";
	cv::Mat depth_image = cv::imread(fileName, cv::IMREAD_UNCHANGED);
	FiveSet params = dia.FindSolution((float*)depth_image.data, 164*164, 164, 164, true, bbox, intrinsics, 60);
	float currentdt[256*256];
	dia.WriteImage(currentdt, 256*256, 256, 256, params.FirstBest, true, intrinsics);
}
