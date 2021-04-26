import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

class PeTraNet(keras.Model):
    def __init__(self) -> None:
        super(PeTraNet, self).__init__()

        # No max pool.
        self.conv_no_pool_1 = layers.Conv2D(32, 3, padding="same", activation=keras.activations.relu)
        self.conv_no_pool_2 = layers.Conv2D(32, 3, padding="same", activation=keras.activations.relu)

        self.pool1 = layers.MaxPool2D(pool_size=4, strides=2, padding="same")
        self.conv_first_pool_1 = layers.Conv2D(64, 3, padding="same", activation=keras.activations.relu)
        self.conv_first_pool_2 = layers.Conv2D(64, 3, padding="same", activation=keras.activations.relu)

        self.pool2 = layers.MaxPool2D(pool_size=4, strides=4, padding="same")
        self.conv_second_pool_1 = layers.Conv2D(128, 3, padding="same", activation=keras.activations.relu)
        self.conv_second_pool_2 = layers.Conv2D(128, 3, padding="same", activation=keras.activations.relu)

        self.pool3 = layers.MaxPool2D(pool_size=4, strides=4, padding="same")
        self.conv_third_pool_1 = layers.Conv2D(256, 3, padding="same", activation=keras.activations.relu)
        self.conv_third_pool_2 = layers.Conv2D(256, 3, padding="same", activation=keras.activations.relu)

        self.pool4 = layers.MaxPool2D(pool_size=4, strides=4, padding="same")
        self.conv_fourth_pool_1 = layers.Conv2D(512, 3, padding="same", activation=keras.activations.relu)
        self.conv_fourth_pool_2 = layers.Conv2D(512, 3, padding="same", activation=keras.activations.relu)

        self.up1 = layers.UpSampling2D(size=4)
        self.concat1 = layers.Concatenate(axis=-1)
        self.conv_first_up_1 = layers.Conv2D(256, 3, padding="same", activation=keras.activations.relu)
        self.conv_first_up_2 = layers.Conv2D(256, 3, padding="same", activation=keras.activations.relu)

        self.up2 = layers.UpSampling2D(size=4)
        self.concat2 = layers.Concatenate(axis=-1)
        self.conv_second_up_1 = layers.Conv2D(128, 3, padding="same", activation=keras.activations.relu)
        self.conv_second_up_2 = layers.Conv2D(128, 3, padding="same", activation=keras.activations.relu)

        self.up3 = layers.UpSampling2D(size=4)
        self.concat3 = layers.Concatenate(axis=-1)
        self.conv_third_up_1 = layers.Conv2D(64, 3, padding="same", activation=keras.activations.relu)
        self.conv_third_up_2 = layers.Conv2D(64, 3, padding="same", activation=keras.activations.relu)

        self.up4 = layers.UpSampling2D(size=2)
        self.concat4 = layers.Concatenate(axis=-1)
        self.conv_fourth_up_1 = layers.Conv2D(32, 3, padding="same", activation=keras.activations.relu)
        self.conv_fourth_up_2 = layers.Conv2D(32, 3, padding="same", activation=keras.activations.relu)

        self.conv_output = layers.Conv2D(1, 1, padding="same", activation=keras.activations.sigmoid)

    def call(self, inputs, training = None):

        # Downsampling.
        res_no_pool = self.conv_no_pool_1(inputs)
        res_no_pool = self.conv_no_pool_2(res_no_pool)

        res_first_pool = self.pool1(res_no_pool)
        res_first_pool = self.conv_first_pool_1(res_first_pool)
        res_first_pool = self.conv_first_pool_2(res_first_pool)

        res_second_pool = self.pool2(res_first_pool)
        res_second_pool = self.conv_second_pool_1(res_second_pool)
        res_second_pool = self.conv_second_pool_2(res_second_pool)

        res_third_pool = self.pool3(res_second_pool)
        res_third_pool = self.conv_third_pool_1(res_third_pool)
        res_third_pool = self.conv_third_pool_2(res_third_pool)

        res_fourth_pool = self.pool4(res_third_pool)
        res_fourth_pool = self.conv_fourth_pool_1(res_fourth_pool)
        res_fourth_pool = self.conv_fourth_pool_2(res_fourth_pool)

        # Upsampling.
        res_first_up = self.up1(res_fourth_pool)
        res_first_up = self.concat1([res_third_pool, res_first_up])
        res_first_up = self.conv_first_up_1(res_first_up)
        res_first_up = self.conv_first_up_2(res_first_up)

        res_second_up = self.up2(res_first_up)
        res_second_up = self.concat2([res_second_pool, res_second_up])
        res_second_up = self.conv_second_up_1(res_second_up)
        res_second_up = self.conv_second_up_2(res_second_up)

        res_third_up = self.up3(res_second_up)
        res_third_up = self.concat3([res_first_pool, res_third_up])
        res_third_up = self.conv_third_up_1(res_third_up)
        res_third_up = self.conv_third_up_2(res_third_up)

        
        res_fourth_up = self.up4(res_third_up)
        res_fourth_up = self.concat4([res_no_pool, res_fourth_up])
        res_fourth_up = self.conv_fourth_up_1(res_fourth_up)
        res_fourth_up = self.conv_fourth_up_2(res_fourth_up)

        # Result.
        return self.conv_output(res_fourth_up)

    def model(self, shape):
        i = layers.Input(shape, batch_size=1)
        return keras.Model(inputs=i, outputs=self.call(i, False))

        
