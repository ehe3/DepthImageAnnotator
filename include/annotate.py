import depth_image_annotator
import os
import numpy as np
import cv2
import json
import argparse
import math
import time

def options():
    parser = argparse.ArgumentParser(
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--data_dir', type=str,
            default='/home/monocle/Documents/FootSegDataset/exp/', help='The directory to process. This directory should not contain subdirectories')
    parser.add_argument('--display_freq', type=int, default=5, help='How often to print progress to terminal')
    parser.add_argument('--save_image', type=bool, default=True, help='Save the PSO generated images')
    parser.add_argument('--color_ext', type=str, default='_color.png', help='suffix + extension of color images')
    parser.add_argument('--depth_ext', type=str, default='_depth.exr', help='suffix + extension of depth images')
    parser.add_argument('--json_ext', type=str, default='_color.json', help='suffix + extension of json files that contain mask labels')
    parser.add_argument('--ppx', type=float, default=644.489, help='RealSense intrinsics ppx value at time of caputre')
    parser.add_argument('--ppy', type=float, default=358.173, help='RealSense intrinsics ppy value at time of caputre')
    parser.add_argument('--fx', type=float, default=922.249, help='RealSense intrinsics fx value at time of caputre')
    parser.add_argument('--fy', type=float, default=922.439, help='RealSense intrinsics fy value at time of caputre')
    parser.add_argument('--zNear', type=float, default=0.1, help='RealSense intrinsics zNear value at time of caputre')
    parser.add_argument('--zFar', type=float, default=3.0, help='RealSense intrinsics zFar value at time of caputre')
    parser.add_argument('--rs_width', type=int, default=1280, help='RealSense intrinsics width value at time of caputre')
    parser.add_argument('--rs_height', type=int, default=720, help='RealSense intrinsics height value at time of caputre')
    parser.add_argument('--data_width', type=int, default=256, help='Stored data image width')
    parser.add_argument('--data_height', type=int, default=256, help='Stored data image height')
    parser.add_argument('--input_width', type=int, default=128, help='DIA input image width')
    parser.add_argument('--input_height', type=int, default=128, help='DIA input image height')
    
    parser.add_argument('--iterations', type=int, default=100, help='DIA iterations')
    parser.add_argument('--initial_samples', type=int, default=5, help='DIA initial samples')
    parser.add_argument('--iterated_samples', type=int, default=5, help='DIA iterated samples')

    return parser.parse_args()

def json2mask(json_path, label, mask_height=256, mask_width=256):
    img = np.zeros((mask_height, mask_width))
    with open(json_path) as f:
        data = json.load(f)
        shapes = []
        for shape in data["shapes"]:
            if shape["label"]==label:
                shapes.append(shape["points"])
        for shape in shapes:
            polygon = np.array(shape)
            cv2.fillPoly(img, [polygon], 1)
    if len(shapes) == 0:
        return img, False

    return img, True

def get_bbox(mask, enlarge_by=0.3):
    mask = mask.astype(np.uint8)
    if len(mask.shape) == 2:
        height, width = mask.shape
    else:
        height, width, _ = mask.shape
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Find the index of the largest contour
    areas = [cv2.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    cnt=contours[max_index]
    # Get bounding rect of mask. Square. Enlarge
    x,y,w,h = cv2.boundingRect(cnt)
    l=w
    if w>=h:
        l *= (1+enlarge_by) 
    elif w<h:
        l = h * (1+enlarge_by)
    l = round(l)

    if l > width or l > height:
        l = width if (width < height) else height        
        return 0, 0, l

    x -= math.ceil(l * enlarge_by / 2)
    y -= math.ceil(l * enlarge_by / 2)
    if x < 0:
        x = 0
    elif (x+l) > width:
        x = width-l-1
    if y < 0:
        y = 0
    elif (y+l) > height:
        x = height-l-1
    return x,y,l

def annotate(dia, intrinsics, dmap, jpath, opt, is_left=True):
    tmp = os.path.join(opt.data_dir, "tmp.exr")
    save = jpath.replace(opt.json_ext, "_anno_L.png") if is_left else jpath.replace(opt.json_ext, "_anno_R.png")

    label = "Left" if is_left else "Right"
    mask, labelled = json2mask(jpath, label, mask_height=opt.data_width, mask_width=opt.data_height)
    if labelled:
        mask    = cv2.resize(mask, (opt.rs_width, opt.rs_height))
        x,y,l     = get_bbox(mask)
        segmented = dmap * mask
        crop      = segmented[y:(y+l), x:(x+l)] 
        input     = cv2.resize(crop,(opt.input_width, opt.input_height)).astype(np.float32)
        cv2.imwrite(tmp, input)

        bbox = depth_image_annotator.Box(x=x, y=y, width=l)
        params = dia.FindSolution(is_left=is_left, file_name=tmp, bbox=bbox, intrinsics=intrinsics, iterations=opt.iterations, initial_samples=opt.initial_samples, iterated_samples=opt.iterated_samples)
        # Save PSO generated image
        dia.WriteImage(location=save, params=params, is_left=is_left, intrinsics=intrinsics)
        gen = cv2.imread(save, cv2.IMREAD_UNCHANGED)
        gen = 1 - gen
        gen = cv2.resize(gen, (opt.data_width, opt.data_height))
        cv2.imwrite(save, gen*255)
        # clean up
        os.remove(tmp)




if __name__ == '__main__':
    opt = options()
    
    # depth image annotator
    dia = depth_image_annotator.DepthImageAnnotator()
    intrinsics = depth_image_annotator.Intrinsics(ppx=opt.ppx, ppy=opt.ppy, fx=opt.fx, fy=opt.fy, left=0.0, right=opt.rs_width, bottom=opt.rs_height, top=0.0, zNear=opt.zNear, zFar=opt.zFar)

    start = time.time()
    count = 0
    total = 0
    for _, _, filenames in os.walk(opt.data_dir):
        fnames = [f for f in filenames if opt.depth_ext in f]
        total  = len(fnames)
        for f in sorted(fnames):
            # get depth image and resize
            dpath = os.path.join(opt.data_dir, f)
            dmap  = cv2.imread(dpath, cv2.IMREAD_UNCHANGED).astype(np.float32)
            if len(dmap.shape) == 3:
                dmap = dmap[:,:,0]
            dmap  = cv2.resize(dmap, (opt.rs_width, opt.rs_height))

            # get masks, segment, then annotate
            jpath = os.path.join(opt.data_dir, f.replace(opt.depth_ext, opt.json_ext))
            annotate(dia, intrinsics, dmap, jpath, opt, True)  # Left
            annotate(dia, intrinsics, dmap, jpath, opt, False) # Right

            count += 1
            if (count % opt.display_freq) == 0:
                print("Annotated  {}  of  {}  images. ~ {0:.2f} seconds per image".format(count, total, (time.time()-start)/count) )

    duration = time.time() - start
    print("-----------------------------------M--------------------------------------")
    print("Job finished in  {0:.2f}  minutes. On average, {0:.2f} seconds per image.".format(float(duration)/60, float(duration)/count))
