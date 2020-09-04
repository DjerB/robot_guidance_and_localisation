import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":
    print("ENDOSCOPIC DATASET\n")
    for str_rec in tf.python_io.tf_record_iterator('./training_data/endoscopy.tfrecords'):
        example = tf.train.Example()
        example.ParseFromString(str_rec)
        print(dict(example.features.feature).keys())
        string = example.features.feature['image_seq'].bytes_list.value[0]
        output = np.fromstring(string)
        print(output.shape)
        image_seq = tf.decode_raw(example.features.feature['image_seq'], tf.uint8)
        depth_seq = tf.decode_raw(example.features.feature['depth_seq'], tf.float32)  # landmark visibility
        intrinsics = tf.decode_raw(example.features.feature['intrinsics'], tf.float64)  # landmark coordinates

        image_height, image_width, num_views = 240, 320, 10

        image_seq = tf.image.convert_image_dtype(
            tf.reshape(image_seq,
                       [image_height,
                        image_width * num_views, 3]),
            tf.float32)
