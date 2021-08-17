import tensorflow as tf
from tensorflow.python.saved_model import tag_constants
import numpy as np
from show_result import show_test
from dataset_processing import preprocess_input_sample
import time 

test_set = np.load("./dataset/test.npy")

net = tf.saved_model.load("./trained_model", tags=[tag_constants.SERVING])

for sample in test_set:
    sample = preprocess_input_sample(sample)
    prediction = net(sample)
    show_test(sample[0], prediction[0])
    time.sleep(1)