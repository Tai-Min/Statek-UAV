import tensorflow as tf

@tf.function
def parse_sample(input_sample, label_sample):
    # Convert image from 0 - 255 uint8 to 0 - 1 float
    
    input_sample = tf.cast(tf.transpose(input_sample, [1, 2, 0]), tf.float16)
    label_sample = tf.cast(tf.transpose(label_sample, [1, 2, 0]), tf.float16)

    input_sample = tf.image.resize(
        input_sample, [256, 256], method=tf.image.ResizeMethod.BILINEAR, preserve_aspect_ratio=True,
        antialias=False
    )

    label_sample = tf.image.resize(
        label_sample, [256, 256], method=tf.image.ResizeMethod.BILINEAR, preserve_aspect_ratio=True,
        antialias=False
    )

    input_sample = tf.math.divide(input_sample, 255)
    label_sample = tf.math.divide(label_sample, 255)

    input_sample = tf.math.round(input_sample)
    label_sample = tf.math.round(label_sample)

    return(input_sample, label_sample)
