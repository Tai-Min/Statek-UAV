import tensorflow as tf
from net import PeTraNet
import numpy as np
from show_result import show_test
from dataset_processing import preprocess_input_sample
import time 
from tensorflow import keras

test_set = np.load("./dataset/test.npy")

net = PeTraNet()

ckpt = tf.train.Checkpoint(net=net)
manager = tf.train.CheckpointManager(ckpt, './.tf_ckpts', max_to_keep=100)
manager.restore_or_initialize()

for sample in test_set:
    sample = preprocess_input_sample(sample)
    prediction = net(sample)
    show_test(sample[0], prediction[0])
    time.sleep(1)