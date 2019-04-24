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
    return parser.parse_args()

    
class Reviewer(object):
    def __init__(self):
        self.opt = options()
        self.root = Tk()
        self.root.bind_all('<Key>', self.key)
        self.root.title("Review Annotations")
        self.canvas = Canvas(self.root, width=self.opt.image_width*3+2*self.opt.image_spacing, height=self.opt.image_height)
        self.canvas.pack()
        # color_path = os.path.join(self.opt.data_dir, "0000_color.png")
        # self.color_img = ImageTk.PhotoImage(Image.open(color_path))
        # self.slot0 = self.canvas.create_image((0,0),image=self.color_img, anchor=NW)

        self.display()
        self.root.mainloop()

    def key(self, event):
        if event.char == 'a':
            msg = 'Normal Key %r' % event.char
            print(msg)
        else:
            print('Displaying .. ')
            self.change()

    def display(self):
        color_path = os.path.join(self.opt.data_dir, "0000_color.png")
        self.color_img = ImageTk.PhotoImage(Image.open(color_path))
        self.canvas.create_image((0,0), image=self.color_img, anchor=NW)

        annoL_path = os.path.join(self.opt.data_dir, "0000_anno_L.png")
        self.annoL_img = ImageTk.PhotoImage(Image.open(annoL_path))
        self.canvas.create_image((self.opt.image_width+self.opt.image_spacing,0), image=self.annoL_img, anchor=NW)
        
        annoR_path = os.path.join(self.opt.data_dir, "0000_anno_R.png")
        self.annoR_img = ImageTk.PhotoImage(Image.open(annoR_path))
        self.canvas.create_image((2*(self.opt.image_width+self.opt.image_spacing),0), image=self.annoR_img, anchor=NW)
    def change(self):
        color_path = os.path.join(self.opt.data_dir, "0000_anno_L.png")
        self.color_img = ImageTk.PhotoImage(Image.open(color_path))
        self.canvas.create_image((0,0), image=self.color_img, anchor=NW)

if __name__ == '__main__':
    r = Reviewer()
