from tkinter import *
from PIL import ImageTk, Image
import os 
import argparse
from annotate import *
import depth_image_annotator

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
    parser.add_argument('--zFar', type=float, default=10.0, help='RealSense intrinsics zFar value at time of caputre')
    parser.add_argument('--rs_width', type=int, default=1280, help='RealSense intrinsics width value at time of caputre')
    parser.add_argument('--rs_height', type=int, default=720, help='RealSense intrinsics height value at time of caputre')
    parser.add_argument('--iterations', type=int, default=60, help='DIA iterations')
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

        self.editing_pa_params = False
        self.pa_btn_text = StringVar()
        self.pa_btn = Button(self.root, textvariable=self.pa_btn_text, command=self.edit_pa_params)
        self.pa_btn.grid(row=1, column=6)
        self.pa_btn_text.set("Edit")
        self.iter_entry.config(state="disable")

        self.canvas = Canvas(self.root, width=self.opt.image_width*5+4*self.opt.image_spacing, height=self.opt.image_height)
        self.canvas.grid(row=2, columnspan=7)
        self.slot = list()
        for i in range(5):
            self.slot.append(self.canvas.create_image((i*(self.opt.image_width+self.opt.image_spacing),0), anchor=NW))
            
        self.top5_canvas = Canvas(self.root, width=self.opt.image_width*5+4*self.opt.image_spacing, height=self.opt.image_height)
        self.top5_canvas.grid(row=3, columnspan=7)
        self.top5_slot = list()
        for i in range(5):
            self.top5_slot.append(self.top5_canvas.create_image((i*(self.opt.image_width+self.opt.image_spacing),0), anchor=NW))

        self.pa = PsoAnnotator(self.opt.iterations, 
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
            self.editing_pa_params = False
        else:
            self.pa_btn_text.set("Update")
            self.iter_entry.config(state="normal")
            self.editing_pa_params = True

    def write(self, msg):
        self.message.set(msg)
        self.root.update_idletasks()

    def key(self, event):
        if event.char == ' ':
            if not self.pick_left and not self.pick_right:
                self.next()
        elif event.char == 'b':
            if not self.pick_left and not self.pick_right:
                self.prev()
        elif event.char == 'j':
            self.annotate_left()
        elif event.char == 'k':
            self.annotate_right()
        elif event.char == 'h':
            self.delete_left()
        elif event.char == 'l':
            self.delete_right()

        if self.pick_left:
            paramL_path = self.top5_path.replace("_top5L.txt", "_paramL.txt")
            bbox = self.get_bbox_from_params_file(self.top5_path)
            bbox = depth_image_annotator.Box(x=bbox[0], y=bbox[1], width=bbox[2])
            if event.char == '1':
                self.pa.save_anno_text(paramL_path, bbox, self.top5_params[0])
                os.remove(self.top5_path)
                self.pick_left =False 
                self.write(self.prefix)
                self.display(self.index)
            elif event.char == '2':
                self.pa.save_anno_text(paramL_path, bbox, self.top5_params[1])
                os.remove(self.top5_path)
                self.pick_left =False 
                self.write(self.prefix)
                self.display(self.index)
            elif event.char == '3':
                self.pa.save_anno_text(paramL_path, bbox, self.top5_params[2])
                os.remove(self.top5_path)
                self.pick_left =False 
                self.write(self.prefix)
                self.display(self.index)
            elif event.char == '4':
                self.pa.save_anno_text(paramL_path, bbox, self.top5_params[3])
                os.remove(self.top5_path)
                self.pick_left =False 
                self.write(self.prefix)
                self.display(self.index)
            elif event.char == '5':
                self.pa.save_anno_text(paramL_path, bbox, self.top5_params[4])
                os.remove(self.top5_path)
                self.pick_left =False 
                self.write(self.prefix)
                self.display(self.index)
        elif self.pick_right:
            paramR_path = self.top5_path.replace("_top5R.txt", "_paramR.txt")
            bbox = self.get_bbox_from_params_file(self.top5_path)
            bbox = depth_image_annotator.Box(x=bbox[0], y=bbox[1], width=bbox[2])
            if event.char == '1':
                self.pa.save_anno_text(paramR_path, bbox, self.top5_params[0])
                os.remove(self.top5_path)
                self.pick_right =False 
                self.write(self.prefix)
                self.display(self.index)
            elif event.char == '2':
                self.pa.save_anno_text(paramR_path, bbox, self.top5_params[1])
                os.remove(self.top5_path)
                self.pick_right =False 
                self.write(self.prefix)
                self.display(self.index)
            elif event.char == '3':
                self.pa.save_anno_text(paramR_path, bbox, self.top5_params[2])
                os.remove(self.top5_path)
                self.pick_right =False 
                self.write(self.prefix)
                self.display(self.index)
            elif event.char == '4':
                self.pa.save_anno_text(paramR_path, bbox, self.top5_params[3])
                os.remove(self.top5_path)
                self.pick_right =False 
                self.write(self.prefix)
                self.display(self.index)
            elif event.char == '5':
                self.pa.save_anno_text(paramR_path, bbox, self.top5_params[4])
                os.remove(self.top5_path)
                self.pick_right =False 
                self.write(self.prefix)
                self.display(self.index)

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

    def annotate_right(self):
        old_msg = self.message.get()
        self.write("Annotating right foot ... ")
        self.annotate(False)

    def delete_left(self):
        prefix = self.fnames[self.index].replace(self.opt.color_ext, "")
        txt_path = os.path.join(self.opt.data_dir, prefix + "_paramL.txt")
        top5_path = os.path.join(self.opt.data_dir, prefix + "_top5L.txt")
        try:
            if os.path.isfile(txt_path):
                os.remove(txt_path)
            if os.path.isfile(top5_path):
                os.remove(top5_path)
            self.display(self.index)
        except:
            pass

    def delete_right(self):
        prefix = self.fnames[self.index].replace(self.opt.color_ext, "")
        txt_path = os.path.join(self.opt.data_dir, prefix + "_paramR.txt")
        top5_path = os.path.join(self.opt.data_dir, prefix + "_top5R.txt")
        try:
            if os.path.isfile(txt_path):
                os.remove(txt_path)
            if os.path.isfile(top5_path):
                os.remove(top5_path)
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

        # Check if top5 for left/right foot exists
        top5L = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, "_top5L.txt"))
        top5R = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, "_top5R.txt"))
        if os.path.isfile(top5L):
            self.show_top5(top5L, is_left=True)
            self.write("Pick a pose for LEFT foot")
            self.pick_left = True
            self.pick_right = False
        elif os.path.isfile(top5R):
            self.show_top5(top5R, is_left=False)
            self.write("Pick a pose for Right foot")
            self.pick_left = False
            self.pick_right = True
        else:
            self.pick_left = False
            self.pick_right = False
            for i in range(5):
                self.top5_canvas.itemconfig(self.top5_slot[i], image = self.NA)

    def show_top5(self, fpath, is_left=True):
        values = np.loadtxt(fpath) 
        bbox = self.get_bbox_from_params_file(fpath)
        self.top5_path = fpath
        self.top5_params = list()
        self.top5_images = list()
        for i in range(5):
            p = depth_image_annotator.PoseParameters(values[3+11*i], values[3+11*i+1], values[3+11*i+2],
                 values[3+11*i+3], values[3+11*i+4], values[3+11*i+5], values[3+11*i+6],
                 values[3+11*i+7], 
                 values[3+11*i+8], values[3+11*i+9],
                 values[3+11*i+10]
            )
            self.top5_params.append(p)
            img = self.get_pso_crop(p, bbox, is_left)
            img = self.colorize(img)
            self.top5_images.append(ImageTk.PhotoImage(Image.fromarray(img)))
            self.top5_canvas.itemconfig(self.top5_slot[i], image = self.top5_images[i])
        # copied from configure_displays
        f = self.fnames[self.index]
        jpath = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, self.opt.json_ext))
        depth = os.path.join(self.opt.data_dir, f.replace(self.opt.color_ext, self.opt.depth_ext))
        # get resized depth crop
        depth, labelled = self.get_depth_crop(jpath, depth, bbox, is_left)
        if labelled:
            display = ImageTk.PhotoImage(Image.fromarray(self.colorize(depth)))
        else:
            display = self.NA 
        if is_left:
            self.displayL = display
            self.canvas.itemconfig(self.slot[1], image = self.displayL)
        else:
            self.displayR = display
            self.canvas.itemconfig(self.slot[3], image = self.displayR)

    def configure_displays(self, is_left=True):
        # file paths
        f = self.fnames[self.index]
        param_ext = "_paramL.txt" if is_left else "_paramR.txt"
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
        anno = self.get_anno_crop(param, bbox, is_left)
        anno = self.colorize(anno)
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
        anno = cv2.resize(anno, (self.opt.image_width, self.opt.image_height))
        anno[anno==1] = 0 # change background to 0
        anno*=10 # scale by max depth
        return anno
    

    def annotate(self, is_left=True):
        # update PsoAnnotator parameters
        self.pa.update_iterations(int(self.iter_entry.get()))

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
        param_path = (prefix + "_paramL.txt") if is_left else (prefix + "_paramR.txt")
        if os.path.isfile(param_path):
            os.remove(param_path)
        
        self.display(self.index)

    def colorize(self, depth, bins=1000, max_depth=10, offset=0.12):
        mask = depth!=0
        hist = cv2.calcHist([depth.astype('float32')], [0], mask.astype(np.uint8), [bins], [0, max_depth])
        mode = np.argmax(hist) * max_depth / bins 
        max = mode+offset
        min = mode-offset
        depth[depth!=0] = (depth[depth!=0]-min)/(max-min) * 180 + 70

        return depth


if __name__ == '__main__':
    Reviewer()
