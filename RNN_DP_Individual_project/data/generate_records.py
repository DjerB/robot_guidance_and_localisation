from math import cos, sin
import numpy as np
import sys
import argparse
from random import randint
import os
from collections import Counter, namedtuple
from scipy.interpolate import LinearNDInterpolator
import cv2

import tensorflow as tf
from PIL import Image

def depth_read(filename):
    # loads depth map D from png file
    # and returns it as a numpy array,
    # for details see readme.txt

    depth_png = np.array(Image.open(filename), dtype=int)
    # make sure we have a proper 16bit depth map here.. not 8bit!
    assert (np.max(depth_png) > 255)

    depth = depth_png.astype(np.float) / 256.
    depth[depth_png == 0] = -1.
    return depth


def long_image(views, max_views_num):
    height = views[0].image.shape[0]
    width = views[0].image.shape[1]
    total_width = width * max_views_num
    new_im = np.zeros((height, total_width, 3), dtype=np.uint8)
    new_depth = np.zeros((height, total_width), dtype=np.float32)
    new_motion = np.zeros((4, 4 * max_views_num), dtype=np.float32)
    x_offset = 0
    RT_offset = 0

    for view in views:
        print("Depth view shape ", view.depth.shape)
        new_im[:, x_offset:x_offset + width, :] = view.image
        new_depth[:, x_offset:x_offset + width] = view.depth
        x_offset += width

        new_motion[:, RT_offset:RT_offset + 4] = np.reshape(view.P, (4, 4))
        RT_offset += 4

    return new_im, new_depth, new_motion


def _int64_feature(value):
    return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))


def _bytes_feature(value):
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))


def lin_interp(shape, xyd):
    # taken from https://github.com/hunse/kitti
    m, n = shape
    ij, d = xyd[:, 1::-1], xyd[:, 2]
    f = LinearNDInterpolator(ij, d, fill_value=0)
    J, I = np.meshgrid(np.arange(n), np.arange(m))
    IJ = np.vstack([I.flatten(), J.flatten()]).T
    disparity = f(IJ).reshape(shape)
    return disparity


def sub2ind(matrixSize, rowSub, colSub):
    m, n = matrixSize
    return rowSub * (n - 1) + colSub - 1


def generate_depth_map(P_velo2im, velo, im_shape, interp=False, vel_depth=False):
    # load velodyne points and remove all behind image plane (approximation)
    # each row of the velodyne data is forward, left, up, reflectance
    # import pdb;pdb.set_trace()
    velo = velo[velo[:, 2] >= 0, :]
    velo[:, 3] = 1.0

    # import pdb;pdb.set_trace()
    # project the points to the camera
    velo_pts_im = np.dot(P_velo2im, velo.T).T

    velo_pts_im[:, :2] = velo_pts_im[:, :2] / velo_pts_im[:, 2][..., np.newaxis]

    if vel_depth:
        velo_pts_im[:, 2] = velo[:, 0]

    # check if in bounds
    # use minus 1 to get the exact same value as KITTI matlab code
    # velo_pts_im[:, 0] = np.round(velo_pts_im[:,0]) - 1
    # velo_pts_im[:, 1] = np.round(velo_pts_im[:,1]) - 1
    val_inds = (velo_pts_im[:, 0] >= 0) & (velo_pts_im[:, 1] >= 0)
    val_inds = val_inds & (velo_pts_im[:, 0] < im_shape[1]) & (velo_pts_im[:, 1] < im_shape[0])
    velo_pts_im = velo_pts_im[val_inds, :]

    # project to image
    depth = np.zeros((im_shape))
    depth[velo_pts_im[:, 1].astype(np.int), velo_pts_im[:, 0].astype(np.int)] = velo_pts_im[:, 2]

    # find the duplicate points and choose the closest depth
    inds = sub2ind(depth.shape, velo_pts_im[:, 1], velo_pts_im[:, 0])
    dupe_inds = [item for item, count in Counter(inds).items() if count > 1]
    for dd in dupe_inds:
        pts = np.where(inds == dd)[0]
        x_loc = int(velo_pts_im[pts[0], 0])
        y_loc = int(velo_pts_im[pts[0], 1])
        depth[y_loc, x_loc] = velo_pts_im[pts, 2].min()
    depth[depth < 0] = 0

    if interp:
        # interpolate the depth map to fill in holes
        depth_interp = lin_interp(im_shape, velo_pts_im)
        # plt.imsave("test.png", depth_interp, cmap='plasma')
        return depth, depth_interp
    else:
        return depth


def get_image_grid(width, height, fx, fy, cx, cy):
    return np.meshgrid(
        (np.arange(width) - cx) / fx,
        (np.arange(height) - cy) / fy)


def generate_surface(z, width, height, fx, fy, cx, cy):
    S = np.dstack((get_image_grid(width, height, fx, fy, cx, cy) + [np.ones_like(z)])) * z[:, :, np.newaxis]
    pad = np.ones_like(z)
    return np.dstack([S] + [pad]).reshape([-1, 4])


def read_image_depth_from_idx(dataset, idx, resizedwidth, resizedheight):
    image = cv2.resize(np.array(dataset.get_cam3(idx)), (resizedwidth, resizedheight))
    # velo = dataset.get_velo(idx)
    pose = np.dot(dataset.calib.T_cam3_imu, dataset.oxts[idx].T_w_imu)
    # import pdb;pdb.set_trace()
    # depth = generate_depth_map(np.dot(dataset.calib.P_rect_20,dataset.calib.T_cam2_velo), velo, image.shape[:2])
    return image, pose  # ,depth

def load_intrinsics(intrinsics_path):
    intrinsics = np.loadtxt(intrinsics_path)
    return intrinsics

def read_depth(fileName, shape):
    """Load depth file as txt and create corresponding numpy normalised matrix
        Args:
            fileName: path to the file text
            shape: shape of the original frame
        Returns:
            numpy array containing the depth values
    """
    depth = np.loadtxt(fileName).reshape(shape)
    depth = ((depth - depth.min()) * (1/(depth.max() - depth.min()) * 255)).astype('uint8')
    return depth


def dofs_to_transformation_matrix(dofs):
    """
    Convert the 6 DoF into the corresponding transformation matrix
    :param dofs: 6 degrees of freedom, i.e the pose, as a 6D array
    :return: np array representation the transformatio matrix
    """
    alpha = dofs[0]
    beta = dofs[1]
    gamma = dofs[2]
    x = dofs[3]
    y = dofs[4]
    z = dofs[5]

    T = np.zeros((4, 4))

    T[0, 0] = cos(alpha) * cos(beta)
    T[0, 1] = cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma)
    T[0, 2] = cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma)
    T[0, 3] = x

    T[1, 0] = sin(alpha) * cos(beta)
    T[1, 1] = sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma)
    T[1, 2] = sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma)
    T[1, 3] = y

    T[2, 0] = - sin(beta)
    T[2, 1] = cos(beta) * sin(gamma)
    T[2, 2] = cos(beta) * cos(gamma)
    T[2, 3] = z

    T[3, 0] = 0
    T[3, 1] = 0
    T[3, 2] = 0
    T[3, 3] = 1

    return T

def image_resize(image, width=None, height=None, inter=cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    resized = cv2.resize(image, dim, interpolation=inter)

    # return the resized image
    return resized

def random_crop_frame(img, height, width):
    print("before cropping frame ", img.shape)
    range_height = img.shape[0] - height
    range_width = img.shape[1] - width
    y = randint(0, range_height)
    x = randint(0, range_width)
    return img[y:y+height, x:x+width, :], y, x

def crop_depth(depth, height, width, y, x):
    return depth[y:y + height, x:x + width]

def create_record_from_pairs(tfrecordfile, frames_path, depths_path, intrinsics_path, poses_path, initial_height=240, initial_width=320, max_views_num=10):
    """Read a KITTI sequence and write samples to the tfrecordfile

    tfrecordfile: tensorflow data format

    frames_path: str
        base path to the endoscopic frames

    frames_path: str
        base path to the endoscopic depths

    intrinsics_path: str
        base path to the camera intrinsics (txt file)

    poses_path: str
        base path to the poses (.pose file)

    Returns the number of generated groups
    """

    writer = tf.python_io.TFRecordWriter(tfrecordfile)
    resizedheight = 128
    resizedwidth = 416
    generated_groups = 0

    # A tuple to store information for each view
    View = namedtuple('View', {'P', 'K', 'image', 'depth'})

    intrinsics_ori = load_intrinsics(intrinsics_path)

    image = cv2.imread(os.path.join(frames_path, os.listdir(frames_path)[0]))
    image = image_resize(image, width=resizedwidth)
    image, y, x = random_crop_frame(image, resizedheight, resizedwidth)

    ori_height, ori_width = image.shape[:2]

    intrinsics = intrinsics_ori.copy()
    intrinsics[0, 0] = intrinsics_ori[0, 0] * resizedwidth / ori_width
    intrinsics[0, 2] = intrinsics_ori[0, 2] * resizedwidth / ori_width
    intrinsics[1, 1] = intrinsics_ori[1, 1] * resizedheight / ori_height
    intrinsics[1, 2] = intrinsics_ori[1, 2] * resizedheight / ori_height

    homo_intrinsic = np.concatenate([intrinsics, np.zeros([3, 1])], axis=1)
    homo_intrinsic = np.concatenate([homo_intrinsic, np.zeros([1, 4])], axis=0)
    homo_intrinsic[3, 3] = 1.0
    mean_baseline = []

    list_files = os.listdir(frames_path)

    for idx in range(len(list_files)):

        frame_file = os.path.join(frames_path, list_files[idx])

        depth_file = os.path.join(depths_path, list_files[idx][:-4] + ".txt")

        image = cv2.imread(frame_file)
        image = image_resize(image, width=resizedwidth)
        image, y, x = random_crop_frame(image, resizedheight, resizedwidth)

        pose_file = os.path.join(poses_path, "pose" + list_files[idx][6:-4] + ".pose")
        pose = dofs_to_transformation_matrix(np.loadtxt(pose_file))

        depth = read_depth(depth_file, (240, 320))
        depth = image_resize(depth, width=resizedwidth)
        print("depth idx before cropping ", depth.shape)
        depth = crop_depth(depth, resizedheight, resizedwidth, y, x)
        print("depth idx after cropping ", depth.shape)

        #S = generate_surface(depth, ori_width, ori_height, intrinsics_ori[0, 0], intrinsics_ori[1, 1],
        #                     intrinsics_ori[0, 2], intrinsics_ori[1, 2])

        #depth = generate_depth_map(homo_intrinsic, S, image.shape[:2])

        view1 = View(P=pose, K=intrinsics, image=image, depth=depth)
        views = [view1]

        T_pre = pose[0:3, 3]
        R_pre = pose[0:3, 0:3]

        # If there is no more than 10 images afterwards, stop
        if (idx + 9 >= len(list_files)):
            break

        for idx2 in range(idx + 1, len(list_files)):

            frame_file = os.path.join(frames_path, list_files[idx2])

            depth_file = os.path.join(depths_path, list_files[idx2][:-4] + ".txt")

            image = cv2.imread(frame_file)
            image = image_resize(image, width=resizedwidth)
            image, y, x = random_crop_frame(image, resizedheight, resizedwidth)

            pose_file = os.path.join(poses_path, "pose" + list_files[idx2][6:-4] + ".pose")
            pose = dofs_to_transformation_matrix(np.loadtxt(pose_file))

            depth = read_depth(depth_file, (240, 320))
            depth = image_resize(depth, width=resizedwidth)
            print("depth idx2 before cropping ", depth.shape)
            depth = crop_depth(depth, resizedheight, resizedwidth, y, x)
            print("depth idx2 after cropping ", depth.shape)

            #S = generate_surface(depth, ori_width, ori_height, intrinsics_ori[0, 0], intrinsics_ori[1, 1],
            #                     intrinsics_ori[0, 2], intrinsics_ori[1, 2])

            #depth = generate_depth_map(homo_intrinsic, S, image.shape[:2])

            # Check whether the scene is static
            # If motion not big enough between frames, the candidate is dropped
            T_curr = pose[0:3, 3]
            R_curr = pose[0:3, 0:3]
            baseline = np.linalg.norm((-R_pre.transpose().dot(T_pre)) - (-R_curr.transpose().dot(T_curr)))

            if baseline < 0.3:
                continue

            mean_baseline.append(baseline)

            T_pre = T_curr
            R_pre = R_curr

            view2 = View(P=pose, K=intrinsics, image=image, depth=depth)
            views.append(view2)

            if len(views) == max_views_num:
                break

        if len(views) == max_views_num:
            concat_view, concat_depth, concat_motion = long_image(views, max_views_num)
            example = tf.train.Example(features=tf.train.Features(feature={
                'image_seq': _bytes_feature(concat_view.tostring()),
                'depth_seq': _bytes_feature(concat_depth.tostring()),
                'motion_seq': _bytes_feature(concat_motion.tostring()),
                'intrinsics': _bytes_feature(intrinsics.tostring()),
            }))

            print("Saved example")

            writer.write(example.SerializeToString())
            generated_groups += 1

    writer.close()
    return generated_groups

def main():
    parser = argparse.ArgumentParser(description="Train RNN depth")
    parser.add_argument("--frames_dir", type=str, default="./endoscopy/150-1150bmp",
                        help="The path to the frames directory")
    parser.add_argument("--depths_dir", type=str, default="./endoscopy/150-1150depth_corrected_ccx_ccy",
                        help="The path to the depths directory")
    parser.add_argument("--intrinsics_dir", type=str, default="./endoscopy/intrinsics.txt", help="The path to the intrinsics file")
    parser.add_argument("--poses_dir", type=str, default="./endoscopy/poses",
                        help="The path to the intrinsics directory")
    parser.add_argument("--record_name", type=str, default="endoscopy", help="The name of the tf record")
    parser.add_argument("--initial_height", type=int, default=240, help="The frames height")
    parser.add_argument("--initial_width", type=int, default=320, help="The frames width")
    parser.add_argument("--training_data", type=bool, default=True, help="Whether it is a training dataset")

    args = None
    try:
        args = parser.parse_args()
        print(args)
    except:
        print("A problem occured while trying to parse the arguments")
        return 1

    outputdir = "training_data" if args.training_data else "testing_data"

    if not os.path.exists(outputdir):
        os.makedirs(outputdir)

    outfile = os.path.join(outputdir, args.record_name + ".tfrecords")
    create_record_from_pairs(outfile, args.frames_dir, args.depths_dir, args.intrinsics_dir, args.poses_dir, initial_height=args.initial_height, initial_width=args.initial_width)

if __name__ == "__main__":
    sys.exit(main())




