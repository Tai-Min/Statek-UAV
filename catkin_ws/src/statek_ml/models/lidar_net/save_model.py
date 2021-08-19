#!/usr/bin/env python3
import tensorflow as tf
from net import PeTraNet
import numpy as np

tf.keras.backend.set_learning_phase(0)

gpu_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(gpu_devices[0], True)
tf.config.experimental.set_virtual_device_configuration(
            gpu_devices[0],
            [tf.config.experimental.VirtualDeviceConfiguration(
               memory_limit=2 * 1024)])

net = PeTraNet()

ckpt = tf.train.latest_checkpoint('./.tf_ckpts')
tf.train.Checkpoint(net=net).restore(ckpt).expect_partial()

net.predict(tf.zeros((1, 256, 256, 1)))
net.save("./trained_model")

conversion_params = tf.experimental.tensorrt.ConversionParams(
    precision_mode="FP16", max_workspace_size_bytes=int(1 * 1024 * 1024))

converter = tf.experimental.tensorrt.Converter(
    input_saved_model_dir="./trained_model",
    conversion_params=conversion_params)

converter.convert()
def input_fn():
    yield [np.random.normal(size=(1, 256, 256, 1)).astype(np.float32)]
converter.build(input_fn=input_fn)
converter.save("./trained_model_optimized")
