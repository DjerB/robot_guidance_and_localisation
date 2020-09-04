# Robot guidance and localisation inside the human body

MSc in AI Individual project repository

![Framework](https://github.com/DjerB/robot_guidance_and_localisation/blob/master/framework.png?raw=true)

The present code aims at estimating depth and pose of endoscopic frames. To compensate the lack of available data, a pipeline is proposed to generate a custom dataset from a single patient's 3D CT chest model.

The proposed pipeline is divided into 3 stages:

# 3D to 2D registration (Stage 1)

The code is entirely provided by Mali Shen et al. [[1]](#1). It creates ground truth depth maps from CT model.

<tt>main.cpp</tt> in <tt>reg3d2d/src</tt> requires manually defined pose and 3D CT model path.

# CycleGAN for generating frames from depth maps

<tt>CycleGAN_for_generating_frames.ipynb</tt> provides the steps to convert depth txt files from stage 1 and optionally use corresponding GT frames to train a CycleGAN model. The notebook is built upon Zhu et al.' work and their implementation [implementation](https://github.com/junyanz/pytorch-CycleGAN-and-pix2pix)

# RNN DP for estimating depth and relative pose using generated dataset and intrinsics and poses

<tt>rnn_dp/data/generate_records.py</tt> generates Tensorflow serialised records before training. It takes the GT frames and depths and poses directories and the camera intrinsics relative paths to create sequences of 10 consecutive frames. 

<tt>rnn_dp/data/dataset.py</tt> deserialises the Tf records, performs data augmentation on pairs of frames and depths, divide them into batches and creates the dataset. 

<tt>rnn_dp/main.py</tt> takes the tf records path as input, calls <tt>dataset.py</tt> to create the dataset and set up the model and computational graph.

<tt>trainer.py</tt> provides the training functions (losses, dataloader, train) and saves the model.

<tt>rnn_dp</tt> is mostly based on Wang et al.'s [implementation](https://github.com/wrlife/RNN_depth_pose) of their work.

## References
<a id="1">[1]</a> 
Mali Shen and Yun Gu and Ning Liu and Guang-Zhong Yang (2019). 
Context-Aware Depth and Pose Estimation for Bronchoscopic Navigation, IEEE IEEE Robotics and Automation Letters

<a id="2">[2]</a>
Zhu, Jun-Yan and Park, Taesung and Isola, Phillip and Efros, Alexei A (2017).
Unpaired Image-to-Image Translation using Cycle-Consistent Adversarial Networks, IEEE

<a id="3">[3]</a>
Wang, Rui and Pizer, Stephen M and Frahm, Jan-Michael (2019)
Recurrent Neural Network for (Un-) supervised Learning of Monocular Video Visual Odometry and Depth, EEE Conference on Computer Vision and Pattern Recognition
