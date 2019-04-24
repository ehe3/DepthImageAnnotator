from tkinter import *
from PIL import ImageTk, Image
import os 
import argparse

def options():
    parser = argparse.ArgumentParser(
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--data_dir', type=str,
            default='/home/monocle/Documents/FootSegDataset/exp/', help='The directory to process. This directory should not contain subdirectories')
    parser.add_argument('--image_width', type=int, default=256, help='The width of each displayed image.')
    parser.add_argument('--image_height', type=int, default=256, help='The height of each displayed image.')
    parser.add_argument('--image_spacing', type=int, default=2, help='Gap between each displayed image.')
    parser.add_argument('--color_ext', type=str, default='_color.png', help='suffix + extension of color images')
    parser.add_argument('--depth_ext', type=str, default='_depth.exr', help='suffix + extension of depth images')
    parser.add_argument('--json_ext', type=str, default='_color.json', help='suffix + extension of json files that contain mask labels')
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
        self.canvas = Canvas(self.root, width=self.opt.image_width*3+2*self.opt.image_spacing, height=self.opt.image_height)
        self.canvas.pack()
        self.slot = list()
        for i in range(5):
            self.slot.append(self.canvas.create_image((i*(self.opt.image_width+self.opt.image_spacing),0), anchor=NW))

        self.display(self.index)
        self.root.mainloop()

    def key(self, event):
        if event.char == ' ':
            self.next()
        elif event.char == 'b':
            self.prev()
            
    def next(self):
        self.index += 1
        if self.index >= len(self.fnames):
            self.index-=1
            print("Finished Reviewing ! ")
        else:
            self.display(self.index)

    def prev(self):
        if self.index != 0:
            self.index -= 1
            self.display(self.index)

    def display(self, index):
        f = self.fnames[index]
        color_path = os.path.join(self.opt.data_dir, f)
        self.color_img = ImageTk.PhotoImage(Image.open(color_path))
        self.canvas.create_image((0,0), image=self.color_img, anchor=NW)
        self.canvas.itemconfig(self.slot[0], image = self.color_img)

        annoL_path = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, "_anno_L.png"))
        self.annoL_img = ImageTk.PhotoImage(Image.open(annoL_path))
        self.canvas.itemconfig(self.slot[1], image = self.annoL_img)
        
        annoR_path = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, "_anno_R.png"))
        self.annoR_img = ImageTk.PhotoImage(Image.open(annoR_path))
        self.canvas.itemconfig(self.slot[2], image = self.annoR_img)

        self.root.title(color_path)

if __name__ == '__main__':
    r = Reviewer()
