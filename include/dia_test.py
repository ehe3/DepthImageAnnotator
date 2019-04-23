import depth_image_annotator

if __name__ == '__main__':
    dia = depth_image_annotator.DepthImageAnnotator()

    intrinsics = depth_image_annotator.Intrinsics(ppx=635.246, ppy=374.675, fx=923.48, fy=924.343, left=0.0, right=1280.0, bottom=720.0, top=0.0, zNear=0.1, zFar=3.0)
    bbox = depth_image_annotator.Box(x=12, y=0, width=533)

    # is_left: true if left foot, false if right foot
    # file_name: location of exr file to be processed, assumed to be 128x128
    # bbox: bounding box of image
    # intrinsics: camera intrinsics to construct projection matrix
    # iterations: number of generations PSO is run for
    # initial_samples: number of times PSO is called independently, the best pose out of the independent sampling is then chosen for iterative sampling
    # iterated_samples: number of times PSO is called with the previous best pose being passed as sampling input to PSO, these samples are called after the initial sampling
    params = dia.FindSolution(is_left=False, file_name='/home/eric/Dev/DepthImageAnnotator/res/depth_image.exr', bbox=bbox, intrinsics=intrinsics, iterations=100, initial_samples=100, iterated_samples=5)
    params.Print();

    # location: output exr file location
    # params: poseparameters to be rendered
    dia.WriteImage(location="/home/eric/Dev/DepthImageAnnotator/res/out_depth_image.exr", params=params, is_left=True, intrinsics=intrinsics)
