import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from math import sqrt


class PeTraNet(keras.Model):
    def __init__(self) -> None:
        super(PeTraNet, self).__init__()

        # N describes standard deviation for weight initiaization
        # based on Gaussian distribution.
        # and is computed as kernel_width * kernel_height * num_features.
        # See unet's paper for details.
        N = 1

        channels_divider = 2

        # Downsampling part.
        self.conv_no_pool_1 = layers.Conv2D(
            32 // channels_divider,
            3,
            padding="same",
            activation=keras.activations.relu,
            kernel_initializer=keras.initializers.RandomNormal(
                stddev=sqrt(2/N))
        )

        N = 9 * 32
        self.conv_no_pool_2 = layers.Conv2D(
            32 // channels_divider,
            3,
            padding="same",
            activation=keras.activations.relu,
            kernel_initializer=keras.initializers.RandomNormal(
                stddev=sqrt(2/N))
        )

        self.pool1 = layers.MaxPool2D(pool_size=4, strides=2, padding="same")
        N = 9 * 32
        self.conv_first_pool = layers.Conv2D(
            64 // channels_divider,
            3,
            padding="same",
            activation=keras.activations.relu,
            kernel_initializer=keras.initializers.RandomNormal(
                stddev=sqrt(2/N))
        )

        self.pool2 = layers.MaxPool2D(pool_size=4, strides=4, padding="same")
        N = 9 * 64
        self.conv_second_pool = layers.Conv2D(
            128 // channels_divider,
            3,
            padding="same",
            activation=keras.activations.relu,
            kernel_initializer=keras.initializers.RandomNormal(
                stddev=sqrt(2/N))
        )

        self.pool3 = layers.MaxPool2D(pool_size=4, strides=4, padding="same")
        N = 9 * 128
        self.conv_third_pool = layers.Conv2D(
            256 // channels_divider,
            3,
            padding="same",
            activation=keras.activations.relu,
            kernel_initializer=keras.initializers.RandomNormal(
                stddev=sqrt(2/N))
        )

        # Upsampling part.
        self.up1 = layers.UpSampling2D(size=4)
        self.concat1 = layers.Concatenate(axis=-1)
        N = 9 * 256
        self.conv_first_up = layers.Conv2D(
            128 // channels_divider,
            3,
            padding="same",
            activation=keras.activations.relu,
            kernel_initializer=keras.initializers.RandomNormal(
                stddev=sqrt(2/N))
        )

        self.up2 = layers.UpSampling2D(size=4)
        self.concat2 = layers.Concatenate(axis=-1)
        N = 9 * 128
        self.conv_second_up = layers.Conv2D(
            64 // channels_divider,
            3,
            padding="same",
            activation=keras.activations.relu,
            kernel_initializer=keras.initializers.RandomNormal(
                stddev=sqrt(2/N))
        )

        self.up3 = layers.UpSampling2D(size=2)
        self.concat3 = layers.Concatenate(axis=-1)
        N = 9 * 64
        self.conv_third_up = layers.Conv2D(
            32 // channels_divider,
            3,
            padding="same",
            activation=keras.activations.relu,
            kernel_initializer=keras.initializers.RandomNormal(
                stddev=sqrt(2/N))
        )

        N = 9 * 32
        self.conv_output = layers.Conv2D(
            1,
            1,
            padding="same",
            activation=keras.activations.linear,
            kernel_initializer=keras.initializers.RandomNormal(
                stddev=sqrt(2/N))
        )

    @tf.function(jit_compile=True)
    def downsample(self, inputs):
        res_no_pool = self.conv_no_pool_1(inputs)
        res_no_pool = self.conv_no_pool_2(res_no_pool)

        res_first_pool = self.pool1(res_no_pool)
        res_first_pool = self.conv_first_pool(res_first_pool)

        res_second_pool = self.pool2(res_first_pool)
        res_second_pool = self.conv_second_pool(res_second_pool)

        res_third_pool = self.pool3(res_second_pool)
        res_third_pool = self.conv_third_pool(res_third_pool)

        return res_no_pool, res_first_pool, res_second_pool, res_third_pool

    @tf.function(jit_compile=True)
    def conv_up_first(self, res_second_pool, res_second_up):
        res_second_up = self.concat1([res_second_pool, res_second_up])
        res_second_up = self.conv_first_up(res_second_up)
        return res_second_up

    @tf.function(jit_compile=True)
    def conv_up_second(self, res_first_pool, res_third_up):
        res_third_up = self.concat2([res_first_pool, res_third_up])
        res_third_up = self.conv_second_up(res_third_up)
        return res_third_up

    @tf.function(jit_compile=True)
    def conv_up_third(self, res_no_pool, res_fourth_up):
        res_fourth_up = self.concat3([res_no_pool, res_fourth_up])
        res_fourth_up = self.conv_third_up(res_fourth_up)
        return self.conv_output(res_fourth_up)

    @tf.function
    def call(self, inputs, training=False):

        # Downsampling.
        res_no_pool, res_first_pool, res_second_pool, res_third_pool = self.downsample(
            inputs)

        # Upsampling.
        res_first_up = self.up1(res_third_pool)
        res_first_up = self.conv_up_first(res_second_pool, res_first_up)

        res_second_up = self.up2(res_first_up)
        res_second_up = self.conv_up_second(res_first_pool, res_second_up)

        res_third_up = self.up3(res_second_up)
        result = self.conv_up_third(res_no_pool, res_third_up)

        if not training:
            result = tf.nn.sigmoid(result)

        return result
