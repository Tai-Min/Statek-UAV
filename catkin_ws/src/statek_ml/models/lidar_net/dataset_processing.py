import tensorflow as tf
import numpy as np
import cv2
import imutils
import math
import os
import shutil
import random

from tensorflow.python.ops.gen_array_ops import fill

def _get_legs(label):
    # @brief Extract legs from given binary label.
    # @param label Binary image u8c1 where 0 - empty space and ~255 - leg.
    # @return List of legs as list of pairs [y,x] where each pairs describes center coordinates of one leg.

    label_squeezed = np.squeeze(label.copy())

    cnts = cv2.findContours(
        label_squeezed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cnts = imutils.grab_contours(cnts)

    legs = []

    for c in cnts:
        M = cv2.moments(c)

        # There are no legs in this label.
        if M["m00"] == 0:
            continue

        # Compute the center of the contour.
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])

        coords = [y, x]
        legs.append(coords)

    return legs


def _get_distances(y, x, legs):
    # @brief Get list of euclidean distances from given pixel [y,x] to each leg.
    # @param y Y coordinate of pixel.
    # @param x X coordinate of pixel.
    # @return list of euclidean distances to each leg.

    distances = []
    for leg in legs:
        leg_x = leg[1]
        leg_y = leg[0]
        d = math.sqrt(
            math.pow(leg_x - x, 2) +
            math.pow(leg_y - y, 2)
        )
        distances.append(d)
    return distances


def _get_leg_weights_for_label(height, width, legs, w0, sigma):
    # @brief Get matrix with weights computed based on euclidean distance from each pixel to closes leg.
    # This function is a modification of original unet's implementation of distance based on
    # distance to border of two cells.
    # @param height Height of processed image.
    # @param width Width of processed image.
    # @param legs List of leg coordinates acquired from _get_legs.
    # @param w0 Tuning parameter. See unet's paper for details.
    # @param sigma Tuning parameter. See unet's paper for details.
    # @return Matrix with equal shape to label's containing weights.

    den = 2 * sigma * sigma

    weight_matrix = np.zeros([height, width], dtype=np.float32)

    for y in range(height):
        for x in range(width):
            distances = _get_distances(y, x, legs)

            if len(distances) == 0:
                d1 = math.sqrt(
                    math.pow(width, 2) +
                    math.pow(height, 2)
                ) * 2

            else:
                d1 = min(distances)

            weight = w0 * math.exp(-(math.pow(d1, 2))/(den))
            weight_matrix[y, x] = weight

    return weight_matrix


def _get_class_weights_for_label(label):
    # @brief Get weight matrix to balance class inequality.
    # @param label Label to generate weight matrix for.
    # Return Weigh matrix with class weights.

    white_pixels = np.count_nonzero(label)
    total_pixels = label.shape[0] * label.shape[1]

    black_weight = white_pixels / total_pixels
    white_weight = 1.0 - black_weight

    weight_matrix = np.where(label > 0, white_weight, black_weight)
    return weight_matrix


def _get_weights_for_label(label, height, width, legs, w0, sigma):
    # @brief Generate weight matrix for class equalizing and distance from legs.
    # @param label Label to generate weights for.
    # @param height Height of processed image.
    # @param width Width of processed image.
    # @param legs List of leg coordinates acquired from _get_legs.
    # @param w0 Tuning parameter. See unet's paper for details.
    # @param sigma Tuning parameter. See unet's paper for details.
    # @return Matrix with equal shape to label's containing weights.

    class_weights = _get_class_weights_for_label(label)
    leg_weights = _get_leg_weights_for_label(height, width, legs, w0, sigma)

    return class_weights + leg_weights


def _generate_weights(train_labels, w0, sigma):
    # @brief Generate weights for all labels.
    # @param w0 Tuning parameter. See unet's paper for details.
    # @param sigma Tuning parameter. See unet's paper for details.
    # @return Numpy array with weight matrices.

    train_legs_weights = []

    cnt = 1
    num_labels = len(train_labels)
    for label in train_labels:
        width = label.shape[2]
        height = label.shape[1]
        legs = _get_legs(label)
        train_legs_weights.append(_get_weights_for_label(
            label, height, width, legs, w0, sigma))

        print("Processed sample %d of %d." % (cnt, num_labels))
        cnt += 1

    return np.array(train_legs_weights)


def _preprocess_inputs_labels(train_inputs, train_labels):
    # @brief Preprocess inputs and labels from uint8 (0 - 255) to float32 (0 - 1).
    # @param train_inputs Inputs to process.
    # @param train_labels Labels to process.
    # @return preprocessed inputs and labels.

    train_inputs_processed = np.zeros(train_inputs.shape)
    train_labels_processed = np.zeros(train_labels.shape)

    num_labels = len(train_labels)
    for i in range(len(train_inputs)):
        input_sample = np.ndarray.astype(train_inputs[i], np.float32)
        label_sample = np.ndarray.astype(train_labels[i], np.float32)

        input_sample = input_sample / 255.0
        label_sample = label_sample / 255.0

        input_sample = np.round(input_sample)
        label_sample = np.round(label_sample)

        train_inputs_processed[i] = input_sample
        train_labels_processed[i] = label_sample
        print("%d of %d inputs and labels processed." % (i+1, num_labels))

    return train_inputs_processed, train_labels_processed


def _clear_single_folder(folder):
    # @brief Remove all files and symlinks from given folder.
    # @param folder String with path to folder.

    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print('Failed to delete %s. Reason: %s' % (file_path, e))


def _clear_dataset_folders():
    # @brief Clear folders for inputs, labels and weights.

    _clear_single_folder("./dataset/inputs")
    _clear_single_folder("./dataset/labels")
    _clear_single_folder("./dataset/weights")


def preprocess_dataset():
    # @brief Preprocess whole dataset and save it
    # into npy files (each for one sample / label / weight).

    print("Preprocessing dataset...")

    train_inputs = np.load("./dataset/train_global_points.npy")
    train_labels = np.load("./dataset/train_global_labels.npy")

    # Remove strange artifact at first pixel from train inputs.
    print("Fixing artifacts in train_inputs...")
    for train_input in train_inputs:
        train_input[0, 0] = 0

    # Generate weights for legs.
    print("Generating weights...")
    train_weights = _generate_weights(train_labels, 10, 5)

    # Process inputs and labels so these are 0 and 1 instead of 0 and 255.
    print("Processing inputs and labels...")
    train_inputs, train_labels = _preprocess_inputs_labels(
        train_inputs, train_labels)

    print("Cleaning dataset folders.")
    _clear_dataset_folders()

    print("Saving new dataset...")
    for i in range(len(train_inputs)):
        np.save("./dataset/inputs/%d.npy" % i, train_inputs[i])
        np.save("./dataset/labels/%d.npy" % i, train_labels[i])
        np.save("./dataset/weights/%d.npy" % i, train_weights[i])
        print("%d.npy saved!" % i)

    print("Data preprocessed.")


def parse_sample(sample):
    # @brief Callback for dataset map function. 
    # Use given sample path to load input, label and weight.
    # @param sample Path to sample from Dataset.from_files().
    # @return Tuple of input, label and weight tensors.

    sample = bytes.decode(sample.numpy())
    sample = os.path.basename(sample)

    input_sample = np.load("./dataset/inputs/%s" % sample)
    label_sample = np.load("./dataset/labels/%s" % sample)
    weights_sample = np.load("./dataset/weights/%s" % sample)

    input_sample = np.ndarray.astype(input_sample, np.float32)
    label_sample = np.ndarray.astype(label_sample, np.float32)
    weights_sample = np.ndarray.astype(weights_sample, np.float32)

    # Apply data augumentation.
    rotation = random.uniform(0, 6.28) 
    shift_x = random.uniform(-0.2, 0.2) * input_sample.shape[1]
    shift_y = random.uniform(-0.2, 0.2) * input_sample.shape[0]
    shear = random.uniform(-3, 3)
    zoom_x = random.uniform(0.7, 1.3)
    zoom_y = random.uniform(0.7, 1.3)

    input_sample = tf.keras.preprocessing.image.apply_affine_transform(input_sample, rotation,
    shift_x, shift_y, shear, zoom_x, zoom_y, fill_mode='constant', cval=0, row_axis=1, col_axis=2, channel_axis=0)
    label_sample = tf.keras.preprocessing.image.apply_affine_transform(label_sample, rotation,
    shift_x, shift_y, shear, zoom_x, zoom_y, fill_mode='constant', cval=0, row_axis=1, col_axis=2, channel_axis=0)
    weights_sample = tf.keras.preprocessing.image.apply_affine_transform(weights_sample, rotation,
    shift_x, shift_y, shear, zoom_x, zoom_y, fill_mode='nearest', row_axis=1, col_axis=2, channel_axis=0)

    input_sample = tf.transpose(input_sample, (1, 2, 0))
    label_sample = tf.transpose(label_sample, (1, 2, 0))
    weights_sample = tf.transpose(weights_sample, (1, 2, 0))

    return (input_sample, label_sample, weights_sample)

def preprocess_input_sample(sample):
    sample = np.ndarray.astype(sample, np.float32)
    sample = sample / 255.0
    sample = np.round(sample)

    sample = tf.transpose(sample, (1, 2, 0))
    sample = tf.expand_dims(sample, axis=0)
    return sample