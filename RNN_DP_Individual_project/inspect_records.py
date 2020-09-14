import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from trainer import *


if __name__ == "__main__":
    print("ENDOSCOPIC DATASET\n")
    # for str_rec in tf.python_io.tf_record_iterator('endoscopy_backup.tfrecords'):
    #     example = tf.train.Example()
    #     example.ParseFromString(str_rec)
    #     #print(dict(example.features.feature).keys())
    #     string = example.features.feature['image_seq'].bytes_list.value[0]
    #     output = np.fromstring(string)
    #     print(output.shape)
        # image_seq = tf.decode_raw(example.features.feature['image_seq'], tf.uint8)
        # depth_seq = tf.decode_raw(example.features.feature['depth_seq'], tf.float32)  # landmark visibility
        # intrinsics = tf.decode_raw(example.features.feature['intrinsics'], tf.float64)  # landmark coordinates

        #image_height, image_width, num_views = 128, 416, 10

        # image_seq = tf.image.convert_image_dtype(
        #     tf.reshape(image_seq,
        #                [image_height,
        #                 image_width * num_views, 3]),
        #     tf.float32)
    m_trainer = RNN_depth_trainer()
    image_height, image_width, num_views = 128, 416, 10
    # Initialize data loading object
    dataLoader = m_trainer.initDataloader("./data/training_data", num_epochs=20)

    #with tf.variable_scope(tf.get_variable_scope()) as outer_scope:


    config = tf.ConfigProto(allow_soft_placement=True)

    with tf.Session(config=config) as sess:
        try:
            count = 0
            while count < 3:
                count += 1
                #sess.run(tf.local_variables_initializer())
                #sess.run(tf.global_variables_initializer())
                data_dict = m_trainer.load_data(dataLoader)
                image_seq = data_dict['image_seq']
                depth_seq = data_dict['depth_seq']

                image = tf.slice(image_seq,
                                 [0, 0, 416 * count, 0],
                                 [-1, -1, 416, -1])

                image.set_shape([1, image_height, image_width, 3])
                print("image ", image.shape)

                depth = tf.slice(depth_seq,
                                 [0, 0, 416 * count, 0],
                                 [-1, -1, 416, -1])
                depth.set_shape([1, image_height, image_width, 1])

                print("depth ", depth.shape)

                res = sess.run([image, depth])

                res_seq = sess.run([image_seq, depth_seq])

                whole_seq = sess.run([image_seq])
                whole_depth = sess.run([depth_seq])

                print(res[0].shape)

                img = np.squeeze(res[0][0])
                depth = np.squeeze(res[1][0])

                fig, axs = plt.subplots(2, 1)

                #plt.imshow(img)
                #axs[0].imshow(np.squeeze(whole_depth[0][0]))
                #axs[1].imshow(whole_seq[0][0][:,:,::-1])
                axs[0].imshow(res_seq[0][0][:,:, ::-1])
                axs[1].imshow(np.squeeze(res_seq[1][0]))
                print(np.min(np.squeeze(res_seq[1][0])), " - ", np.max(np.squeeze(res_seq[1][0])))
                plt.show()


        except tf.errors.OutOfRangeError as e:
            print("End of sequence")

        #image.set_shape([1, 128, 416, 3])
