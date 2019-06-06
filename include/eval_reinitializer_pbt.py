import sys
import depth_image_annotator
from tkinter import *
from PIL import ImageTk, Image
import os 
import argparse
from annotate import *

def json2mask(json_path, label, mask_height=480, mask_width=640):
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

def options():
    parser = argparse.ArgumentParser(
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--data_dir', type=str,
            default='/home/monocle/Documents/40k/val', help='The directory to process. This directory should not contain subdirectories')
    parser.add_argument('--ppx', type=float, default=644.489, help='RealSense intrinsics ppx value at time of caputre')
    parser.add_argument('--ppy', type=float, default=358.173, help='RealSense intrinsics ppy value at time of caputre')
    parser.add_argument('--fx', type=float, default=922.249, help='RealSense intrinsics fx value at time of caputre')
    parser.add_argument('--fy', type=float, default=922.439, help='RealSense intrinsics fy value at time of caputre')
    parser.add_argument('--zNear', type=float, default=0.1, help='RealSense intrinsics zNear value at time of caputre')
    parser.add_argument('--zFar', type=float, default=10.0, help='RealSense intrinsics zFar value at time of caputre')
    parser.add_argument('--rs_width', type=int, default=1280, help='RealSense intrinsics width value at time of caputre')
    parser.add_argument('--rs_height', type=int, default=720, help='RealSense intrinsics height value at time of caputre')
    parser.add_argument('--start', type=int, default=0, help='Image number to start from.')
    parser.add_argument('--image_spacing', type=int, default=3, help='Gap between each displayed image.')
    parser.add_argument('--iterations', type=int, default=60, help='DIA iterations')
    return parser.parse_args()


class Reviewer(object):

    def __init__(self):
        self.opt = options()
        self.root_dir         = self.opt.data_dir
        self.depth_ext        = "_depth.exr"
        self.json_ext         = "_color.json"
        self.paramL_ext       = "_paramL.txt"
        self.paramR_ext       = "_paramR.txt"
        self.shape            = (128, 128)
        self.param_paths = list()


        # load paths to paramL and paramR text files
        sub_dirs = next(os.walk(self.root_dir))[1]
        sub_dirs += [""]
        for dir in sub_dirs:
            cur_dir = os.path.join(self.root_dir, dir) 
            with os.scandir(cur_dir) as folder:
                for entry in folder:
                    if not entry.name.startswith('.') and entry.is_file():
                        if entry.name.endswith(self.paramL_ext) or entry.name.endswith(self.paramR_ext):
                            self.param_paths.append(os.path.join(cur_dir, entry.name))

        self.index = self.opt.start
        self.root = Tk()
        self.root.bind_all('<Key>', self.key)
        self.root.title("be watah my friend")
        self.pa = PsoAnnotator(self.opt.iterations, 
            self.opt.ppx, self.opt.ppy, self.opt.fx, self.opt.fy, 
            self.opt.rs_width, self.opt.rs_height, self.opt.zNear, self.opt.zFar
        )

        self.canvas = Canvas(self.root, width=self.shape[1]*3+2*self.opt.image_spacing, height=self.shape[0])
        self.canvas.pack()
        self.slot = list()
        for i in range(3):
            self.slot.append(self.canvas.create_image((i*(self.shape[1]+self.opt.image_spacing),0), anchor=NW))
        self.NA = ImageTk.PhotoImage(Image.new("RGB", self.shape, (248, 24, 148))) # Use this image when something is not available

        self.display(self.index)
        self.root.mainloop()

    def display(self, index):
        param_path = self.param_paths[index]
        print(param_path)
        if "L" in param_path:
            is_left = True
            prefix  = param_path.replace(self.paramL_ext, "")
        else:
            is_left = False
            prefix  = param_path.replace(self.paramR_ext, "")
        depth_path = prefix + self.depth_ext
        json_path  = prefix + self.json_ext

        params = np.loadtxt(param_path)
        bb = (int(params[0]), int(params[1]), int(params[2]))

        # input crop visualization
        input_crop = self.get_depth_crop(json_path, depth_path, bb, is_left)
        self.input_crop = ImageTk.PhotoImage(Image.fromarray(self.colorize(input_crop)))
        self.canvas.itemconfig(self.slot[0], image = self.input_crop)

        # gt visualization
        gt = self.get_anno_crop(param_path, bb, is_left)
        self.gt = ImageTk.PhotoImage(Image.fromarray(self.colorize(gt)))
        self.canvas.itemconfig(self.slot[1], image = self.gt)



    def get_depth_crop(self, json_path, depth_path, bb, is_left=True):
        if is_left:
            mask, labelled = json2mask(json_path, "Left", mask_height=480, mask_width=640)
        else:
            mask, labelled = json2mask(json_path, "Right", mask_height=480, mask_width=640)
        if labelled:
            mask = cv2.resize(mask, (self.opt.rs_width, self.opt.rs_height))
            depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            depth = cv2.resize(depth, (self.opt.rs_width, self.opt.rs_height))
            depth = depth * mask

            bb_x = bb[0]
            bb_y = bb[1]
            bb_l = bb[2]
            depth = depth[bb_y:(bb_y+bb_l), bb_x:(bb_x+bb_l)]
            depth = cv2.resize(depth, self.shape)
            return depth
        else:
            return self.NA

    def get_anno_crop(self, param_path, bb, is_left):
        params = np.loadtxt(param_path) 
        params = depth_image_annotator.PoseParameters(params[3], params[4], params[5],
                params[6], params[7], params[8], params[9],
                params[10], 
                params[11], params[12],
                params[13]
        )
        return self.get_pso_crop(params, bb, is_left)
    
    def get_pso_crop(self, params, bb, is_left):
        bb_x = bb[0]
        bb_y = bb[1]
        bb_l = bb[2]
        anno = self.pa.draw_pso_image(params, is_left, w=self.opt.rs_width, h=self.opt.rs_height)
        anno = anno[bb_y:(bb_y+bb_l), bb_x:(bb_x+bb_l)]
        anno = cv2.resize(anno, self.shape)
        anno[anno==1] = 0 # change background to 0
        anno*=10 # scale by max depth
        return anno

    def key(self, event):
        print("Yo")

    def colorize(self, depth, bins=1000, max_depth=10, offset=0.3):
        mask = depth!=0
        hist = cv2.calcHist([depth.astype('float32')], [0], mask.astype(np.uint8), [bins], [0, max_depth])
        mode = np.argmax(hist) * max_depth / bins 
        max = mode+offset
        min = mode-offset
        depth[depth!=0] = (depth[depth!=0]-min)/(max-min) * 255

        return depth
if __name__ == '__main__':
    Reviewer()
        


