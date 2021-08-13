import tensorflow as tf
from net import PeTraNet

net = PeTraNet()

ckpt = tf.train.latest_checkpoint('./.tf_ckpts')
ckpt = tf.train.Checkpoint(net=net).restore(ckpt).expect_partial()

net.predict(tf.zeros((1, 512, 512, 1)))
net.save("./trained_model")

conversion_params = tf.experimental.tensorrt.ConversionParams(
    precision_mode="FP16")

converter = tf.experimental.tensorrt.Converter(
    input_saved_model_dir="./trained_model",
    conversion_params=conversion_params)

converter.convert()

converter.save("./trained_model")
