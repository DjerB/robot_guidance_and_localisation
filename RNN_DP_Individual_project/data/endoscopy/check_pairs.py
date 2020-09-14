import numpy as np
import os
import cv2
import matplotlib.pyplot as plt

def read_depth(fileName, shape):
	depth = np.loadtxt(fileName).reshape(shape)
	depth = ((depth - depth.min()) * (1/(depth.max() - depth.min()) * 255)).astype('uint8')
	return depth

frames = os.listdir("150-1150bmp")
depths = os.listdir("150-1150depth_corrected_ccx_ccy")
nb_pairs = len(frames)

fix, axs = plt.subplots(2, 10, figsize=(40, 20))
for i in range(nb_pairs):
	if i >= 10:
		break
	frame = cv2.imread(os.path.join("150-1150bmp", frames[i]))
	depth = read_depth((os.path.join("150-1150depth_corrected_ccx_ccy", frames[i][:-4] + ".txt")), (240, 320))
	axs[0, i].imshow(frame)
	axs[1, i].imshow(depth)

plt.show()
