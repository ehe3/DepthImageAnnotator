#include "depth_image_annotator.h"

int main()
{
	Intrinsics intrinsics(635.246, 374.675, 923.48, 924.343, 0.0, 1280.0, 720.0, 0.0, 0.1, 3.0);
	Box bbox(697, 7, 460);
	DepthImageAnnotator dia;
	PoseParameters params = dia.FindSolution(true, "/home/eric/Dev/DepthImageAnnotator/res/depth_image.exr", bbox, intrinsics, 500, 5, 5);
	params.Print();
	dia.WriteImage("/home/eric/Dev/DepthImageAnnotator/res/out_depth_image.exr", params, true, intrinsics);
}
