import tensorflow as tf
import cv2
import numpy as np


def _result_to_cv_img(img, is_prediction=False):
    if is_prediction:
        img = tf.nn.sigmoid(img)

    img = img.numpy() * 255
    img = img.astype(np.uint8)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

def _insert_text(img, text):
    return cv2.putText(img, text, (4, 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255),
                            1, cv2.LINE_AA, False)

def show_result(input_img, output_img, pred_img, weight_img):
    if show_result.first_iter:
        cv2.namedWindow("Train progress", cv2.WINDOW_AUTOSIZE)
        show_result.first_iter = False

    input_img = _result_to_cv_img(input_img)
    input_img = _insert_text(input_img, "Input image")

    output_img = _result_to_cv_img(output_img)
    output_img = _insert_text(output_img, "Label image")

    pred_img = _result_to_cv_img(pred_img, True)
    pred_img = _insert_text(pred_img, "Network's prediction")

    weight_img = _result_to_cv_img(weight_img, True)
    weight_img_map = cv2.applyColorMap(weight_img, cv2.COLORMAP_JET)
    weight_img_map = _insert_text(weight_img_map, "Weight heatmap")

    img_top = np.concatenate((input_img, output_img), axis=1)
    img_bot = np.concatenate((pred_img, weight_img_map), axis=1)
    img = np.concatenate((img_top, img_bot), axis=0)

    cv2.imshow("Train progress", img)
    cv2.waitKey(10)


show_result.first_iter = True

def show_test(input_img, pred_img):
    if show_test.first_iter:
        cv2.namedWindow("Test image", cv2.WINDOW_AUTOSIZE)
        show_test.first_iter = False

    input_img = _result_to_cv_img(input_img)
    input_img = _insert_text(input_img, "Input image")

    pred_img = _result_to_cv_img(pred_img)
    pred_img = _insert_text(pred_img, "Network's prediction")

    img = np.concatenate((input_img, pred_img), axis=1)

    cv2.imshow("Test image", img)
    cv2.waitKey(10)

show_test.first_iter = True