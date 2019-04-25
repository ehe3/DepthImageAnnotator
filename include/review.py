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
        self.root.title("be watah my friend")
        self.message = StringVar()
        self.msg_label = Label(self.root, textvariable=self.message, font=("Arial", 16))
        self.msg_label.grid(row=0, column=1, columnspan=5)
        self.message.set("Monocle Vision")

        self.progress = StringVar()
        self.progress_label = Label(self.root, textvariable=self.progress)
        self.progress_label.grid(row=0, column=6)
        self.progress.set("0 %")
        
        self.total = Label(self.root, text="{}".format(len(self.fnames)))
        self.total.grid(row=0, column=0)

        self.iter_label = Label(self.root, text="Iterations:")
        self.iter_label.grid(row=1, column=0)
        self.iter_entry = Entry(self.root, bd =1)
        self.iter_entry.grid(row=1, column=1)
        self.iter_entry.insert(0, str(self.opt.iterations))

        self.INS_label = Label(self.root, text="Initial Samples:")
        self.INS_label.grid(row=1, column=2)
        self.INS_entry = Entry(self.root, bd =1)
        self.INS_entry.grid(row=1, column=3)
        self.INS_entry.insert(0, str(self.opt.initial_samples))
        
        self.ITS_label = Label(self.root, text="Iterated Samples:")
        self.ITS_label.grid(row=1, column=4)
        self.ITS_entry = Entry(self.root, bd =1)
        self.ITS_entry.grid(row=1, column=5)
        self.ITS_entry.insert(0, str(self.opt.iterated_samples))

        self.editing_pa_params = False
        self.pa_btn_text = StringVar()
        self.pa_btn = Button(self.root, textvariable=self.pa_btn_text, command=self.edit_pa_params)
        self.pa_btn.grid(row=1, column=6)
        self.pa_btn_text.set("Edit")
        self.iter_entry.config(state="disable")
        self.INS_entry.config(state="disable")
        self.ITS_entry.config(state="disable")

        self.canvas = Canvas(self.root, width=self.opt.image_width*5+4*self.opt.image_spacing, height=self.opt.image_height)
        self.canvas.grid(row=2, columnspan=7)
        self.slot = list()
        for i in range(5):
            self.slot.append(self.canvas.create_image((i*(self.opt.image_width+self.opt.image_spacing),0), anchor=NW))

        self.pa = PsoAnnotator(self.opt.iterations, self.opt.initial_samples, self.opt.iterated_samples, 
            self.opt.ppx, self.opt.ppy, self.opt.fx, self.opt.fy, 
            self.opt.rs_width, self.opt.rs_height, self.opt.zNear, self.opt.zFar
        )
        self.NA = ImageTk.PhotoImage(Image.new("RGB", (self.opt.image_width, self.opt.image_height), (248, 24, 148))) # Use this image when something is not available
        self.display(self.index)
        self.root.mainloop()

    def edit_pa_params(self):
        if self.editing_pa_params:
            self.pa_btn_text.set("Edit")
            self.iter_entry.config(state="disable")
            self.INS_entry.config(state="disable")
            self.ITS_entry.config(state="disable")
            self.editing_pa_params = False
        else:
            self.pa_btn_text.set("Update")
            self.iter_entry.config(state="normal")
            self.INS_entry.config(state="normal")
            self.ITS_entry.config(state="normal")
            print(self.editing_pa_params)
            self.editing_pa_params = True

    def write(self, msg):
        self.message.set(msg)
        self.root.update_idletasks()

    def key(self, event):
        if event.char == ' ':
            self.next()
        elif event.char == 'b':
            self.prev()
        elif event.char == 'j':
            self.annotate_left()
        elif event.char == 'k':
            self.annotate_right()
        elif event.char == 'h':
            self.delete_left()
        elif event.char == 'l':
            self.delete_right()

    def refresh_progress(self):
        progress = self.index / float(len(self.fnames)) * 100 
        progress = format(progress, '.1f')
        self.progress.set(" {} %".format(progress))

    def next(self):
        self.index += 1
        if self.index >= len(self.fnames):
            self.index-=1
            self.write("You are all done !! ")
            self.progress.set("100 %")
        else:
            self.display(self.index)
            self.refresh_progress()

    def prev(self):
        if self.index != 0:
            self.index -= 1
            self.display(self.index)
        self.refresh_progress()


    def annotate_left(self):
        old_msg = self.message.get()
        self.write("Annotating left foot ... ")
        self.annotate(True)
        self.write(old_msg)

    def annotate_right(self):
        old_msg = self.message.get()
        self.write("Annotating right foot ... ")
        self.annotate(False)
        self.write(old_msg)

    def delete_left(self):
        prefix = self.fnames[self.index].replace(self.opt.color_ext, "")
        txt_path = os.path.join(self.opt.data_dir, prefix + "_paramL.txt")
        png_path = os.path.join(self.opt.data_dir, prefix + "_annoL.png")
        try:
            os.remove(txt_path)
            os.remove(png_path)
            self.display(self.index)
        except:
            pass

    def delete_right(self):
        prefix = self.fnames[self.index].replace(self.opt.color_ext, "")
        txt_path = os.path.join(self.opt.data_dir, prefix + "_paramR.txt")
        png_path = os.path.join(self.opt.data_dir, prefix + "_annoR.png")
        try:
            os.remove(txt_path)
            os.remove(png_path)
            self.display(self.index)
        except:
            pass


    def display(self, index):
        # slot 0 - color
        f = self.fnames[index]
        self.prefix = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, ""))
        self.write(self.prefix)
        color_path = os.path.join(self.opt.data_dir, f)
        self.color_img = ImageTk.PhotoImage(Image.open(color_path))
        self.canvas.itemconfig(self.slot[0], image = self.color_img)

        # Left:  slot 1 and 2
        paramL = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, "_paramL.txt"))
        if os.path.isfile(paramL):
            self.configure_displays(True)
        else:
            self.canvas.itemconfig(self.slot[1], image = self.NA)
            self.canvas.itemconfig(self.slot[2], image = self.NA)

        # Right: slot 3 and 4
        paramR = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, "_paramR.txt"))
        if os.path.isfile(paramR):
            self.configure_displays(False)
        else:
            self.canvas.itemconfig(self.slot[3], image = self.NA)
            self.canvas.itemconfig(self.slot[4], image = self.NA)

    def configure_displays(self, is_left=True):
        # file paths
        f = self.fnames[self.index]
        param_ext = "_paramL.txt" if is_left else "_paramR.txt"
        anno_ext = "_annoL.png" if is_left else "_annoR.png"
        param = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, param_ext))
        jpath = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, self.opt.json_ext))
        depth = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, self.opt.depth_ext))
        # get bounding box from param file
        bbox = self.get_bbox_from_params_file(param)
        # get resized depth crop
        depth, labelled = self.get_depth_crop(jpath, depth, bbox, is_left)
        if labelled:
            display = ImageTk.PhotoImage(Image.fromarray(self.colorize(depth)))
        else:
            display = self.NA 
        # get resized pso crop
        anno = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, anno_ext))
        anno = self.get_anno_crop(anno, bbox)
        # display
        if is_left:
            self.displayL = display
            self.canvas.itemconfig(self.slot[1], image = self.displayL)
            self.annoL = ImageTk.PhotoImage(Image.fromarray(anno))
            self.canvas.itemconfig(self.slot[2], image = self.annoL)
        else:
            self.displayR = display
            self.canvas.itemconfig(self.slot[3], image = self.displayR)
            self.annoR = ImageTk.PhotoImage(Image.fromarray(anno))
            self.canvas.itemconfig(self.slot[4], image = self.annoR)


    def get_bbox_from_params_file(self, fpath):
        params = np.loadtxt(fpath)
        bb_x = int(params[0]) 
        bb_y = int(params[1])
        bb_l = int(params[2])
        return (bb_x, bb_y, bb_l)
    
    def get_depth_crop(self, json_path, depth_path, bb, is_left=True):
        if is_left:
            mask, labelled = json2mask(json_path, "Left", mask_height=self.opt.image_height, mask_width=self.opt.image_width)
        else:
            mask, labelled = json2mask(json_path, "Right", mask_height=self.opt.image_height, mask_width=self.opt.image_width)
        if labelled:
            mask = cv2.resize(mask, (self.opt.rs_width, self.opt.rs_height))
            depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            depth = cv2.resize(depth, (self.opt.rs_width, self.opt.rs_height))
            depth = depth * mask

            bb_x = bb[0]
            bb_y = bb[1]
            bb_l = bb[2]
            depth = depth[bb_y:(bb_y+bb_l), bb_x:(bb_x+bb_l)]
            depth = cv2.resize(depth, (self.opt.image_width, self.opt.image_height))
            return depth, labelled
        else:
            return self.NA, labelled

    def get_anno_crop(self, anno_path, bb):
        try:
            anno = cv2.imread(anno_path, 0)
        except:
            return self.NA
        anno = cv2.resize(anno, (self.opt.rs_width, self.opt.rs_height))
        bb_x = bb[0]
        bb_y = bb[1]
        bb_l = bb[2]
        anno = anno[bb_y:(bb_y+bb_l), bb_x:(bb_x+bb_l)]
        anno = cv2.resize(anno, (self.opt.image_width, self.opt.image_height))
        return anno

    def annotate(self, is_left=True):
        # update PsoAnnotator parameters
        self.pa.update_iterations(int(self.iter_entry.get()))
        self.pa.update_initial_samples(int(self.INS_entry.get()))
        self.pa.update_iterated_samples(int(self.ITS_entry.get()))

        # label, save prefix, and file paths
        label = "Left" if is_left else "Right"
        prefix = self.fnames[self.index].replace(self.opt.color_ext, "")
        prefix = os.path.join(self.opt.data_dir, prefix)
        dpath = prefix + self.opt.depth_ext
        jpath = prefix + self.opt.json_ext
        # Grab map and mask
        dmap  = cv2.imread(dpath, cv2.IMREAD_UNCHANGED).astype(np.float32)
        if len(dmap.shape) == 3:
            dmap = dmap[:,:,0]
        dmap  = cv2.resize(dmap, (self.opt.rs_width, self.opt.rs_height))
        mask, labelled = json2mask(jpath, label, mask_height=self.opt.image_width, mask_width=self.opt.image_height)
        if labelled:
            mask = cv2.resize(mask, (self.opt.rs_width, self.opt.rs_height))
            self.pa.annotate(dmap, mask, prefix, is_left)
        self.display(self.index)

    def colorize(self, depth, bins=1000, max_depth=10, offset=0.12):
        mask = depth!=0
        hist = cv2.calcHist([depth.astype('float32')], [0], mask.astype(np.uint8), [bins], [0, max_depth])
        mode = np.argmax(hist) * max_depth / 1000 
        max = mode+offset
        min = mode-offset
        depth[depth!=0] = (depth[depth!=0]-min)/(max-min) * 180 + 70

        return depth


if __name__ == '__main__':
    Reviewer()
