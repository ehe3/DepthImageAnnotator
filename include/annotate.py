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
    parser.add_argument('--zFar', type=float, default=10.0, help='RealSense intrinsics zFar value at time of caputre')
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
    x,y,l = square_bb((x,y,w,h), (width,height))
    x,y,l = enlarge_square((x,y,l), (width,height))

    return x,y,l

def square_bb(bb, im_size):
    x = bb[0]
    y = bb[1]
    w = bb[2]
    h = bb[3]
    im_w = im_size[0]
    im_h = im_size[1]
    len = 0
    diff = 0 
    if (w < h):
        len = h
        diff = h - w
        x -= round(diff * 0.5)
        if (x < 0):
            x = 0
        elif ((x + len) >= im_w):
            x = (im_w - 1 - len)
    elif (w > h):
        len = w
        diff = w - h
        y -= round(diff * 0.5)
        if (y < 0):
            y = 0
        elif ((y + len) >= im_h):
            y = (im_h - 1 - len)
    else:
        return x,y,w
    return x, y, len

def enlarge_square(sq, im_size, by=0.3):
    x = sq[0]
    y = sq[1]
    l = sq[2]
    im_w = im_size[0]
    im_h = im_size[1]

    l_new = round(l * (1 + by))
    if (l_new >= im_w or l_new >= im_h):
        return x,y,l

    offset = round((l_new - l) / 2)

    x -= offset
    if (x < 0):
        x = 0
    elif ((x+l_new) >= im_w):
        x = im_w-l_new-1

    y -= offset
    if (y < 0):
        y = 0
    elif ((y+l_new) >= im_h):
        y = im_h-l_new-1
    return x,y,l_new



class PsoAnnotator(object):
    def __init__(self, iterations, initial_samples, iterated_samples, ppx, ppy, fx, fy, width, height, zNear, zFar, input_l=128):
        self.dia = depth_image_annotator.DepthImageAnnotator()
        self.intrinsics = depth_image_annotator.Intrinsics(ppx=ppx, ppy=ppy, fx=fx, fy=fy, left=0.0, right=width, bottom=height, top=0.0, zNear=zNear, zFar=zFar)

        self.iterations = iterations
        self.initial_samples = initial_samples
        self.iterated_samples = iterated_samples
        self.input_l = input_l

    def annotate(self, dmap, mask, save_prefix, is_left=True, save_size=(256,256)):
        x,y,l     = get_bbox(mask)
        segmented = dmap * mask
        crop      = segmented[y:(y+l), x:(x+l)] 
        input     = cv2.resize(crop,(self.input_l, self.input_l)).astype(np.float32)
        self.annotate_crop(crop, x, y, l, save_prefix, is_left, save_size)

    def annotate_crop(self, crop, bb_x, bb_y, bb_l, save_prefix, is_left=True, save_size=(256,256)):
        # process
        w = crop.shape[1]
        h = crop.shape[0]
        crop = crop.astype(np.float32).ravel()
        bbox = depth_image_annotator.Box(x=bb_x, y=bb_y, width=bb_l)
        params = self.dia.FindSolution(depth_data=crop, w=w, h=h,
                is_left=is_left, bbox=bbox, 
                intrinsics=self.intrinsics, iterations=self.iterations, 
                initial_samples=self.initial_samples, iterated_samples=self.iterated_samples
        )
        # write bbox info and params to a file
        txt_path = str(save_prefix + "_paramL.txt") if is_left else str(save_prefix + "_paramR.txt")
        self.save_anno_text(txt_path, bbox, params)

    # def save_pso_image(self, path, params, is_left=True, save_size=(256,256)):
        # self.dia.WriteImage(location=path, params=params, is_left=is_left, intrinsics=self.intrinsics)
        # gen = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        # gen = 1 - gen
        # gen = cv2.resize(gen, save_size)
        # cv2.imwrite(path, gen*255)

    def draw_pso_image(self, params, is_left=True, w=256, h=256): 
        depth_array = dia.WriteImage(currentdt=w*h, w=w, h=h, params=params, is_left=is_left, intrinsics=self.intrinsics)
        depth_array = depth_array.reshape((h,w))
        return depth_array

    def save_anno_text(self, path, bbox, params):
        with open(path, 'w') as f:
            f.write("%s " % bbox.x)
            f.write("%s " % bbox.y)
            f.write("%s " % bbox.width)
            f.write("%s " % params.XTranslation)
            f.write("%s " % params.YTranslation)
            f.write("%s " % params.ZTranslation)
            f.write("%s " % params.GetQuatW)
            f.write("%s " % params.GetQuatX)
            f.write("%s " % params.GetQuatY)
            f.write("%s " % params.GetQuatZ)
            f.write("%s " % params.ToeXRot)
            f.write("%s " % params.LegXRot)
            f.write("%s " % params.LegZRot)

    def update_iterations(self, iter):
        self.iterations = iter
    def update_initial_samples(self, ins):
        self.initial_samples = ins
    def update_iterated_samples(self, its):
        self.iterated_samples = its

if __name__ == '__main__':
    opt = options()
    
    # depth image annotator
    pa = PsoAnnotator(opt.iterations, opt.initial_samples, opt.iterated_samples, 
            opt.ppx, opt.ppy, opt.fx, opt.fy, 
            opt.rs_width, opt.rs_height, opt.zNear, opt.zFar
         )

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
            save_prefix = jpath.replace(opt.json_ext, "") 
            # Left
            mask, labelled = json2mask(jpath, "Left", mask_height=opt.data_width, mask_width=opt.data_height)
            if labelled:
                mask = cv2.resize(mask, (opt.rs_width, opt.rs_height))
                pa.annotate(dmap, mask, save_prefix, True)

            # Right
            mask, labelled = json2mask(jpath, "Right", mask_height=opt.data_width, mask_width=opt.data_height)
            if labelled:
                mask = cv2.resize(mask, (opt.rs_width, opt.rs_height))
                pa.annotate(dmap, mask, save_prefix, False)

            count += 1
            if (count % opt.display_freq) == 0:
                print("Annotated  {}  of  {}  images. {} seconds per image".format(count, total, (time.time()-start)/count) )

    duration = time.time() - start
    print("\n---------------------------------  M  -----------------------------------")
    print("Job finished in  {0:.2f}  minutes. On average, {0:.2f} seconds per image.".format(float(duration)/60, float(duration)/count))
