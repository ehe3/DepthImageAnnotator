from tkinter import *
from PIL import ImageTk, Image
import os 
import argparse
from annotate import *

def options():
    parser = argparse.ArgumentParser(
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--data_dir', type=str,
            default='/home/monocle/Documents/FootSegDataset/exp/', help='The directory to process. This directory should not contain subdirectories')
    parser.add_argument('--image_width', type=int, default=256, help='The width of each displayed image.')
    parser.add_argument('--image_height', type=int, default=256, help='The height of each displayed image.')
    parser.add_argument('--image_spacing', type=int, default=3, help='Gap between each displayed image.')
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
    parser.add_argument('--iterations', type=int, default=100, help='DIA iterations')
    parser.add_argument('--initial_samples', type=int, default=30, help='DIA initial samples')
    parser.add_argument('--iterated_samples', type=int, default=5, help='DIA iterated samples')
    return parser.parse_args()

    
class Reviewer(object):
    def __init__(self):
        self.opt = options()
        for _, _, filenames in os.walk(self.opt.data_dir):
            self.fnames = [f for f in filenames if self.opt.color_ext in f]
        self.index = 0
        self.root = Tk()
        self.root.bind_all('<Key>', self.key)
        self.root.title("Review Annotations")
        self.canvas = Canvas(self.root, width=self.opt.image_width*5+4*self.opt.image_spacing, height=self.opt.image_height)
        self.canvas.pack()
        self.slot = list()
        for i in range(5):
            self.slot.append(self.canvas.create_image((i*(self.opt.image_width+self.opt.image_spacing),0), anchor=NW))

        self.display(self.index)
        self.pa = PsoAnnotator(self.opt.iterations, self.opt.initial_samples, self.opt.iterated_samples, 
            self.opt.ppx, self.opt.ppy, self.opt.fx, self.opt.fy, 
            self.opt.rs_width, self.opt.rs_height, self.opt.zNear, self.opt.zFar
        )
        self.root.mainloop()

    def key(self, event):
        if event.char == ' ':
            self.next()
        elif event.char == 'b':
            self.prev()
        elif event.char == 'j':
            self.annotate_left()
        elif event.char == 'k':
            self.annotate_right()
            
    def next(self):
        self.index += 1
        if self.index >= len(self.fnames):
            self.index-=1
            print("You are all done !! ")
        else:
            self.display(self.index)

    def prev(self):
        if self.index != 0:
            self.index -= 1
            self.display(self.index)

    def annotate_left(self):
        save_prefix = self.fnames[self.index].replace(self.opt.color_ext, "")
        save_prefix = os.path.join(self.opt.data_dir, save_prefix)
        print("annotating left")
        input = cv2.resize(self.depthL, (128,128))
        self.pa.annotate_crop(input, self.bboxL[0], self.bboxL[1], self.bboxL[2], save_prefix, True, (self.opt.image_width,self.opt.image_height))
        print("finished")
        self.display(self.index)

    def annotate_right(self):
        save_prefix = self.fnames[self.index].replace(self.opt.color_ext, "")
        save_prefix = os.path.join(self.opt.data_dir, save_prefix)
        print("annotating right")
        input = cv2.resize(self.depthR, (128,128))
        self.pa.annotate_crop(input, self.bboxR[0], self.bboxR[1], self.bboxR[2], save_prefix, False, (self.opt.image_width,self.opt.image_height))
        print("finished")
        self.display(self.index)

    def display(self, index):
        # slot 0 - color
        f = self.fnames[index]
        color_path = os.path.join(self.opt.data_dir, f)
        self.color_img = ImageTk.PhotoImage(Image.open(color_path))
        self.canvas.itemconfig(self.slot[0], image = self.color_img)

        # slot 1 - left depth crop
        paramL = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, "_paramL.txt"))
        paramL = np.loadtxt(paramL)
        bb_x = int(paramL[0]) 
        bb_y = int(paramL[1])
        bb_l = int(paramL[2])
        self.bboxL = (bb_x, bb_y, bb_l)
        print("BBOX LEFT")
        print(self.bboxL)
        jpath = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, self.opt.json_ext))
        mask, labelled = json2mask(jpath, "Left", mask_height=self.opt.image_width, mask_width=self.opt.image_height)
        if labelled:
            mask = cv2.resize(mask, (self.opt.rs_width, self.opt.rs_height))
            depthL = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, self.opt.depth_ext))
            depthL = cv2.imread(depthL, cv2.IMREAD_UNCHANGED)
            depthL = cv2.resize(depthL, (self.opt.rs_width, self.opt.rs_height))
            depthL = depthL * mask
            depthL = depthL[bb_y:(bb_y+bb_l), bb_x:(bb_x+bb_l)]
            self.depthL = cv2.resize(depthL, (self.opt.image_width, self.opt.image_height))
            print(self.depthL.shape)
            self.depthL_display = ImageTk.PhotoImage(Image.fromarray(self.depthL*255))
            print(self.depthL_display.width())
            print(self.depthL_display.height())
            self.canvas.itemconfig(self.slot[1], image = self.depthL_display)
        else:
            # display pink image to indicate no crop available
            print("not lablled")


        # slot 2 - left anno crop (use same bbox as above)
        annoL = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, "_annoL.png"))
        annoL = cv2.imread(annoL, 0)
        annoL = cv2.resize(annoL, (self.opt.rs_width, self.opt.rs_height))
        annoL = annoL[bb_y:(bb_y+bb_l), bb_x:(bb_x+bb_l)]
        annoL = cv2.resize(annoL, (self.opt.image_width, self.opt.image_height))
        self.annoL = ImageTk.PhotoImage(Image.fromarray(annoL))
        self.canvas.itemconfig(self.slot[2], image = self.annoL)

        # slot 3 - right depth crop
        paramR = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, "_paramR.txt"))
        paramR = np.loadtxt(paramR)
        bb_x = int(paramR[0]) 
        bb_y = int(paramR[1])
        bb_l = int(paramR[2])
        self.bboxR = (bb_x, bb_y, bb_l)
        print("BBOX RIGHT")
        print(self.bboxR)
        jpath = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, self.opt.json_ext))
        mask, labelled = json2mask(jpath, "Right", mask_height=self.opt.image_width, mask_width=self.opt.image_height)
        if labelled:
            mask = cv2.resize(mask, (self.opt.rs_width, self.opt.rs_height))
            depthR = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, self.opt.depth_ext))
            depthR = cv2.imread(depthR, cv2.IMREAD_UNCHANGED)
            depthR = cv2.resize(depthR, (self.opt.rs_width, self.opt.rs_height))
            depthR = depthR * mask
            depthR = depthR[bb_y:(bb_y+bb_l), bb_x:(bb_x+bb_l)]
            self.depthR = cv2.resize(depthR, (self.opt.image_width, self.opt.image_height))
            self.depthR_display = ImageTk.PhotoImage(Image.fromarray(self.depthR*255))
            self.canvas.itemconfig(self.slot[3], image = self.depthR_display)
        else:
            # display pink image to indicate no crop available
            print("not lablled")


        # slot 4 - left anno crop (use same bbox as above)
        annoR = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, "_annoR.png"))
        annoR = cv2.imread(annoR, 0)
        annoR = cv2.resize(annoR, (self.opt.rs_width, self.opt.rs_height))
        annoR = annoR[bb_y:(bb_y+bb_l), bb_x:(bb_x+bb_l)]
        annoR = cv2.resize(annoR, (self.opt.image_width, self.opt.image_height))
        self.annoR = ImageTk.PhotoImage(Image.fromarray(annoR))
        self.canvas.itemconfig(self.slot[4], image = self.annoR)

        self.root.title(color_path)



if __name__ == '__main__':
    Reviewer()
