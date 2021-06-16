#pragma once
#include <geometry_msgs/TransformStamped.h>

/**
 * @brief Get some transform.
 * @param targetFrame Target of transform.
 * @param sourceFrame Source of transform.
 * @param ok Set to true on success, false otherwise.
 * @return If ok then requested transform, otherwise unspecified.
 */
geometry_msgs::TransformStamped getTransform(const std::string &targetFrame, const std::string &sourceFrame, bool &ok);