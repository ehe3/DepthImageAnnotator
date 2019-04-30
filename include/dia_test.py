import depth_image_annotator
import numpy as np
import sys
import cv2

if __name__ == '__main__':
    dia = depth_image_annotator.DepthImageAnnotator()

    intrinsics = depth_image_annotator.Intrinsics(ppx=635.246, ppy=374.675, fx=923.48, fy=924.343, left=0.0, right=1280.0, bottom=720.0, top=0.0, zNear=0.1, zFar=3.0)
    bbox = depth_image_annotator.Box(x=601, y=389, width=77)

    # depth_data: flattened depth image, if input image is 128x128, depth_data dimensions are (128*128,) 
    # w: width of depth data
    # h: height of depth data
    # is_left: true if left foot, false if right foot
    # bbox: bounding box of image
    # intrinsics: camera intrinsics to construct projection matrix
    # iterations: number of generations PSO is run for
    loaded_depth_image = cv2.imread('/home/eric/Desktop/annoying_case5.exr', cv2.IMREAD_UNCHANGED)
    loaded_depth_image = loaded_depth_image.flatten()
    params = dia.FindSolution(depth_data=loaded_depth_image, w=77, h=77, is_left=False, bbox=bbox, intrinsics=intrinsics, iterations=60)
    #params.FirstBest.Print()
    #params.SecondBest.Print()
    #params.ThirdBest.Print()
    #params.FourthBest.Print()
    #params.FifthBest.Print()

    ## currentdt: product of width and height, note this needs to be explicitly passed for swig to know how much memory to allocate, use currendt=1280*720, w=1280, h=720 for default behavior
    ## w: width of output
    ## h: height of output
    ## params: poseparameters to be rendered
    ## intrinsics: original camera intrinsics, they will be scaled in the function call depending on the width and height of the output image provided
    depth_array_1 = dia.WriteImage(currentdt=256*256, w=256, h=256, params=params.FirstBest, is_left=True, intrinsics=intrinsics)
    depth_array_1 = depth_array_1.reshape(256, 256)
    depth_array_2 = dia.WriteImage(currentdt=256*256, w=256, h=256, params=params.SecondBest, is_left=True, intrinsics=intrinsics)
    depth_array_2 = depth_array_2.reshape(256, 256)
    depth_array_3 = dia.WriteImage(currentdt=256*256, w=256, h=256, params=params.ThirdBest, is_left=True, intrinsics=intrinsics)
    depth_array_3 = depth_array_3.reshape(256, 256)
    depth_array_4 = dia.WriteImage(currentdt=256*256, w=256, h=256, params=params.FourthBest, is_left=True, intrinsics=intrinsics)
    depth_array_4 = depth_array_4.reshape(256, 256)
    depth_array_5 = dia.WriteImage(currentdt=256*256, w=256, h=256, params=params.FifthBest, is_left=True, intrinsics=intrinsics)
    depth_array_5 = depth_array_5.reshape(256, 256)
    combined_depth_array = np.hstack((depth_array_1, depth_array_2, depth_array_3, depth_array_4, depth_array_5))

    ## WriteImage() will always return a flat array, need to reshape for 2D image
    #depth_array = depth_array.reshape(256, 256)
    #depth_array_2 = cv2.flip(depth_array_2, 0)
    cv2.imshow('Depth Image', combined_depth_array)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    
    
