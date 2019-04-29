import cv2
import os
import numpy as np
import sys

def get_img(iter_num):
    pic_dir = '/home/eric/Dev/DepthImageAnnotator/include/psoout'
    rep_file_name = 'rep.exr'
    pso_file_name = 'pso_{}.exr'.format(iter_num)
    rep_img = cv2.imread(os.path.join(pic_dir, rep_file_name), cv2.IMREAD_UNCHANGED)
    rep_img = rep_img.reshape(32, 1024)
    pso_img = cv2.imread(os.path.join(pic_dir, pso_file_name), cv2.IMREAD_UNCHANGED)
    pso_img = pso_img.reshape(32, 1024)
    combined_img = np.vstack((rep_img, pso_img))
    return combined_img

np.set_printoptions(threshold=sys.maxsize)
curr_img = 0
img = get_img(curr_img)
while True:
    cv2.imshow('pso', img)
    k = chr(cv2.waitKey())
    if k == 'n':
        if curr_img != 59: 
            curr_img = curr_img + 1
            img = get_img(curr_img)
    elif k == 'b':
        if curr_img != 0:
            curr_img = curr_img - 1
            img = get_img(curr_img)
    elif k == 'q':
        break;

cv2.destroyAllWindows()
