import glob
import tensorflow as tf
import random

def dataset_loader(inputs_path, outputs_path):
    # Get all jpegs.
    input_filenames = glob.glob(inputs_path.decode("utf-8") + "/*.jpg")
    output_filenames = glob.glob(outputs_path.decode("utf-8") + "/*.jpg")
    
    # Shuffle dataset randomly.
    tmp = list(zip(input_filenames, output_filenames))
    random.shuffle(tmp)
    input_filenames, output_filenames = zip(*tmp)

    for input_filename, output_filename in zip(input_filenames, output_filenames):
        yield(input_filename, output_filename)

@tf.function
def load_image(input_file_path, output_file_path):
    input_image = tf.io.read_file(input_file_path)
    input_image = tf.image.decode_jpeg(input_image, channels=1)

    output_image = tf.io.read_file(output_file_path)
    output_image = tf.image.decode_jpeg(output_image, channels=1)

    # Preprocess image into binary matrix with 0 = free space, 1 = obstacle.
    input_image = tf.math.round(tf.math.divide(input_image, 255))
    output_image = tf.math.round(tf.math.divide(output_image, 255))

    return (input_image, output_image)