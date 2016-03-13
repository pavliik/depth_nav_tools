/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 * @file   main.cpp
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @date   11.2015
 * @brief  depth_sensor_pose package
 */

#include <depth_sensor_pose/depth_sensor_pose_node.h>

using namespace depth_sensor_pose;

//==============================================================================
DepthSensorPoseNode::DepthSensorPoseNode(ros::NodeHandle& n, ros::NodeHandle& pnh):
  node_rate_hz_(1), pnh_(pnh), it_(n), srv_(pnh)
{
  boost::mutex::scoped_lock lock(connection_mutex_);
  
  // Set callback for dynamic reconfigure server
  srv_.setCallback(boost::bind(&DepthSensorPoseNode::reconfigureCb, this, _1, _2));
  
  // New depth image publisher
/*  pub_ = it_.advertise("depth_sensor_pose/depth", 1,
                       boost::bind(&DepthSensorPoseNode::connectCb, this),
                       boost::bind(&DepthSensorPoseNode::disconnectCb, this));*/
  pub_ = it_.advertise("depth_sensor_pose/depth", 1);

  // Lazy subscription to depth image topic
  pub_height_ = n.advertise<std_msgs::Float64>(
        "depth_sensor_pose/height", 2,
        boost::bind(&DepthSensorPoseNode::connectCb, this, _1),
        boost::bind(&DepthSensorPoseNode::disconnectCb, this, _1));

  pub_angle_ = n.advertise<std_msgs::Float64>(
        "depth_sensor_pose/tilt_angle", 2,
        boost::bind(&DepthSensorPoseNode::connectCb, this, _1),
        boost::bind(&DepthSensorPoseNode::disconnectCb, this, _1));

}

//==============================================================================
DepthSensorPoseNode::~DepthSensorPoseNode()
{
  sub_.shutdown();
}

//=================================================================================================
void DepthSensorPoseNode::setNodeRate(const unsigned int rate)
{
  if (rate <= 30)
    node_rate_hz_ = rate;
  else
    node_rate_hz_ = 30;
}

//=================================================================================================
unsigned int DepthSensorPoseNode::getNodeRate()
{
  return node_rate_hz_;
}

//==============================================================================
void DepthSensorPoseNode::depthCb( const sensor_msgs::ImageConstPtr& depth_msg,
                                         const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  try
  {
    // Calculate calibration parameters -- sensor pose
    calib_.calibration(depth_msg, info_msg);

    std_msgs::Float64 height, tilt_angle;
    height.data = calib_.getSensorMountHeight();
    tilt_angle.data = calib_.getSensorTiltAngle();

    ROS_ERROR("height = %.4f angle = %.4f", calib_.getSensorMountHeight(), calib_.getSensorTiltAngle());

    pub_height_.publish(height);
    pub_angle_.publish(tilt_angle);

    // Publishes new depth image with added downstairs
    if (calib_.getPublishDepthEnable())
      pub_.publish(calib_.new_depth_msg_);
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR_THROTTLE(1.0, "Could not convert depth image to laserscan: %s", e.what());
  }
}

//==============================================================================
void DepthSensorPoseNode::connectCb(const ros::SingleSubscriberPublisher& pub)
{
  boost::mutex::scoped_lock lock(connection_mutex_);
  if (!sub_ && (pub_height_.getNumSubscribers() > 0 || pub_angle_.getNumSubscribers() > 0
      || pub_.getNumSubscribers() > 0))
  {
    ROS_DEBUG("Connecting to depth topic.");
    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
    sub_ = it_.subscribeCamera("image", 10, &DepthSensorPoseNode::depthCb, this, hints);
  }
}

//==============================================================================
void DepthSensorPoseNode::disconnectCb(const ros::SingleSubscriberPublisher& pub)
{
  boost::mutex::scoped_lock lock(connection_mutex_);
  if (pub_height_.getNumSubscribers() == 0 && pub_angle_.getNumSubscribers() == 0
&& pub_.getNumSubscribers() == 0)
  {
    ROS_DEBUG("Unsubscribing from depth topic.");
    sub_.shutdown();
  }
}

//==============================================================================
void DepthSensorPoseNode::reconfigureCb(
    depth_sensor_pose::DepthSensorPoseConfig& config, uint32_t level)
{
  node_rate_hz_ = (unsigned int) config.rate;

  calib_.setRangeLimits(config.range_min, config.range_max);
  calib_.setSensorMountHeightMin(config.mount_height_min);
  calib_.setSensorMountHeightMax(config.mount_height_max);
  calib_.setSensorTiltAngleMin(config.tilt_angle_min);
  calib_.setSensorTiltAngleMax(config.tilt_angle_max);

  calib_.setPublishDepthEnable(config.publish_depth);
  calib_.setCamModelUpdate(config.cam_model_update);
  calib_.setUsedDepthHeight((unsigned int)config.used_depth_height);
  calib_.setDepthImgStepRow(config.depth_img_step_row);
  calib_.setDepthImgStepCol(config.depth_img_step_col);

  calib_.setRansacDistanceThresh(config.ransac_dist_thresh);
  calib_.setRansacMaxIter(config.ransac_max_iter);
  calib_.setGroundMaxPoints(config.ground_max_points);

  calib_.setReconfParamsUpdated(true);
}
