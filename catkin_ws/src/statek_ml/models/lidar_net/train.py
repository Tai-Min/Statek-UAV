import tensorflow as tf
from tensorflow import keras
from tensorflow._api.v2 import train
from tensorflow.python.keras.backend import dtype
from net import PeTraNet
from dataset_loader import parse_sample
import numpy as np
import cv2

# Training parameters.
epochs = 50000
train_samples_per_epoch = 100
batch_size = 16
init_lr = 0.1
final_lr = 0.0001

# Prepare dataset.
train_inputs = np.load("./dataset/train_global_points.npy")
train_labels = np.load("./dataset/train_global_labels.npy")

train_dataset = tf.data.Dataset.from_tensor_slices(
    (train_inputs, train_labels))
train_dataset = train_dataset.map(parse_sample).shuffle(train_samples_per_epoch * 10).batch(
    batch_size).prefetch(tf.data.experimental.AUTOTUNE)

# Scheduled learning rate.
train_batches_per_epoch = train_samples_per_epoch / batch_size
lr_decay_steps = train_batches_per_epoch * epochs
lr_decay_rate = final_lr / init_lr

lr_schedule = keras.optimizers.schedules.ExponentialDecay(
    init_lr, lr_decay_steps, lr_decay_rate)

# Optimizer.
optimizer = keras.optimizers.SGD(learning_rate=lr_schedule)

# Loss.
loss_fcn = keras.losses.BinaryCrossentropy()

# Metrics.
train_loss_metric = keras.metrics.Mean()

# Network model.
net = PeTraNet()
net.model((256, 256, 1)).summary()

# Restore training from checkpoint if possible
ckpt = tf.train.Checkpoint(step=tf.Variable(1), net=net, optimizer=optimizer)

manager = tf.train.CheckpointManager(ckpt, './.tf_ckpts', max_to_keep=100)
path = manager.restore_or_initialize()

if path:
    print('Restored checkpoint from %s' % path)
else:
    print('Initializing training from scratch.')


def result_to_cv_img(img):
    img = tf.cast(img, tf.float32)
    img = img.numpy()
    img = np.round(img)
    img = np.clip(img, 0.0, 1.0)
    return img


def show_result(input_img, output_img, pred_img):
    if show_result.first_iter:
        cv2.namedWindow("Train progress", cv2.WINDOW_AUTOSIZE)

    input_img = result_to_cv_img(input_img)
    output_img = result_to_cv_img(output_img)
    pred_img = result_to_cv_img(pred_img)

    img = np.concatenate((input_img, output_img, pred_img), axis=1)

    cv2.imshow("Train progress", img)
    cv2.waitKey(10)


show_result.first_iter = True


@tf.function
def train_step(input_imgs, real_outputs):
    with tf.GradientTape() as tape:
        preds = net(input_imgs, True)
        loss = loss_fcn(real_outputs, preds)

    grads = tape.gradient(loss, net.trainable_weights)
    optimizer.apply_gradients(zip(grads, net.trainable_weights))
    train_loss_metric(loss)

    return preds

def train():
    # Perform training.
    for epoch in range(epochs):
        for _ in range(int(train_batches_per_epoch)):
            input_imgs, output_imgs = next(iter(train_dataset))
            preds = train_step(input_imgs, output_imgs)

            # Save checkpoint and show some results.
            ckpt.step.assign_add(1)
            if int(ckpt.step) % 20 == 0:
                pass
                path = manager.save()
                print("Checkpoint saved: %s." % path)
                show_result(input_imgs[0], output_imgs[0], preds[0])

            print("Mean loss on training batch: %f." %
                  float(train_loss_metric.result()))
            train_loss_metric.reset_states()

train()
