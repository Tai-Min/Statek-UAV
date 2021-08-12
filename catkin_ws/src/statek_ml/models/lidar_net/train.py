import tensorflow as tf
from tensorflow import keras
from net import PeTraNet
from dataset_processing import parse_sample, preprocess_dataset
from show_result import show_result

# Configuration.
# Whether to generate correct dataset for this training from
# author's train_global_labels.npy and train_global_points.npy files.
# This may take some time D:
# But speeds up training a lot
# and frees a lot of memory from GPU.
preprocess_dataset_flag = False

# Training parameters.
epochs = 5000
train_samples_per_epoch = 64
batch_size = 1
lr = 0.05

# Preprocess dataset.
# It must be processed as three whole arrays don't fit
# into my GPU's memory ._.
if preprocess_dataset_flag:
    preprocess_dataset()

train_dataset = tf.data.Dataset.list_files("./dataset/inputs/*.npy")

train_dataset = train_dataset.map(
    lambda x: tf.py_function(
        parse_sample, [x], (tf.float32, tf.float32, tf.float32))
).shuffle(train_samples_per_epoch * 4).batch(batch_size)

# Optimizer.
# SGD with momentum as described in unet's paper.
optimizer = keras.optimizers.SGD(learning_rate=lr, momentum=0.99)

# Loss function.


@tf.function(jit_compile=True)
def loss_fcn(labels, preds, weights):
    # @brief Modified function from unet's paper.
    # Replaces softmax with sigmoid and sum with mean for stability.
    # @param labels True labels.
    # @param preds Predictions.
    # @param weights Weights tensors.

    loss = tf.nn.sigmoid_cross_entropy_with_logits(labels, preds)
    loss = tf.multiply(weights, loss)
    loss = tf.reduce_mean(loss)
    return loss


# Network model.
net = PeTraNet()

# Restore training from checkpoint if possible
ckpt = tf.train.Checkpoint(step=tf.Variable(1), net=net, optimizer=optimizer)

manager = tf.train.CheckpointManager(ckpt, './.tf_ckpts', max_to_keep=100)
path = manager.restore_or_initialize()

if path:
    print('Restored checkpoint from %s' % path)
else:
    print('Initializing training from scratch.')


@tf.function()
def train_step(input_imgs, real_outputs, weights):
    # @brief Perform single training step.
    # @param input_imgs Batch of inputs.
    # @param real_outputs Batch of labels.
    # @param weights Batch of weights.
    # @return Predictions and loss function's value.

    with tf.GradientTape() as tape:
        preds = net(input_imgs)
        loss = loss_fcn(real_outputs, preds, weights)

    grads = tape.gradient(loss, net.trainable_weights)
    optimizer.apply_gradients(zip(grads, net.trainable_weights))

    return preds, loss

train_batches_per_epoch = train_samples_per_epoch / batch_size

# Perform training.
for _ in range(epochs):
    for __ in range(int(train_batches_per_epoch)):
        input_imgs, output_imgs, weights = next(iter(train_dataset))
        preds, loss = train_step(input_imgs, output_imgs, weights)

        # Save checkpoint and show some results.
        ckpt.step.assign_add(1)
        if int(ckpt.step) % 100 == 0:
            path = manager.save()
            print("Checkpoint saved: %s." % path)

        show_result(input_imgs[0], output_imgs[0], preds[0], weights[0])

        print("Loss function on training batch: %f." % float(loss))
