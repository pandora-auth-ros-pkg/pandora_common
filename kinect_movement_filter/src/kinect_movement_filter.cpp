/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  Evangelos Apostolidis
*********************************************************************/
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "ros/ros.h"

namespace pandora_common
{
  class KinectMovementFilter
  {
    private:
      ros::NodeHandle nodeHandle_;
      ros::Subscriber jointStateSubscriber_;
      ros::Subscriber imageSubscriber_;
      ros::Subscriber depthImageSubscriber_;
      ros::Subscriber pointCloudSubscriber_;

      ros::Publisher imagePublisher_;
      ros::Publisher depthImagePublisher_;
      ros::Publisher pointCloudPublisher_;

      double previousPitch_;
      double previousYaw_;
      double pitchMovement_;
      double yawMovement_;
      double movementThreshold_;

      void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
      void imageCallback(const sensor_msgs::ImageConstPtr& msg);
      void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
      void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    public:
      KinectMovementFilter();
      ~KinectMovementFilter();
  };

  KinectMovementFilter::KinectMovementFilter()
  {
    if ( nodeHandle_.hasParam("movementThreshold") )
    {
      nodeHandle_.getParam("movementThreshold", movementThreshold_);
      ROS_DEBUG_STREAM(
        "[kinect_movement_filter]: Got parameter movementThreshold : " <<
        movementThreshold_ << std::endl);
    }
    else
    {
      ROS_DEBUG_STREAM(
        "[kinect_movement_filter] : " <<
        "Parameter movementThreshold not found. Using Default" << std::endl);
      movementThreshold_ = 0.017;
    }

    previousPitch_ = 0;
    previousYaw_ = 0;
    pitchMovement_ = movementThreshold_;
    yawMovement_ = movementThreshold_;

    jointStateSubscriber_ = nodeHandle_.subscribe(
      "/joint_states",
      1,
      &KinectMovementFilter::jointStatesCallback,
      this);

    imageSubscriber_ = nodeHandle_.subscribe(
      "/kinect/image",
      1,
      &KinectMovementFilter::imageCallback,
      this);

    depthImageSubscriber_ = nodeHandle_.subscribe(
      "/kinect/depth/image",
      1,
      &KinectMovementFilter::depthImageCallback,
      this);

    pointCloudSubscriber_ = nodeHandle_.subscribe(
      "/kinect/point_cloud",
      1,
      &KinectMovementFilter::pointCloudCallback,
      this);

    imagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(
      "/stable/kinect/image",
      5);

    depthImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(
      "/stable/kinect/depth/image",
      5);

    pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(
      "/stable/kinect/point_cloud",
      5);
  }

  KinectMovementFilter::~KinectMovementFilter()
  {
  }

  void KinectMovementFilter::jointStatesCallback(
    const sensor_msgs::JointStateConstPtr& msg)
  {
    for (int ii = 0; ii < msg->name.size(); ii++)
    {
      if (msg->name[ii] == "kinect_pitch_joint")
      {
        pitchMovement_ = msg->position[ii] - previousPitch_;
        previousPitch_ = msg->position[ii];
      }
      if (msg->name[ii] == "kinect_yaw_joint")
      {
        yawMovement_ = msg->position[ii] - previousYaw_;
        previousYaw_ = msg->position[ii];
      }
    }
  }

  void KinectMovementFilter::imageCallback(
    const sensor_msgs::ImageConstPtr& msg)
  {
    if (pitchMovement_ < movementThreshold_ &&
      yawMovement_ < movementThreshold_)
    {
      imagePublisher_.publish(*msg);
    }
  }

  void KinectMovementFilter::depthImageCallback(
    const sensor_msgs::ImageConstPtr& msg)
  {
    if (pitchMovement_ < movementThreshold_ &&
      yawMovement_ < movementThreshold_)
    {
      depthImagePublisher_.publish(*msg);
    }
  }

  void KinectMovementFilter::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (pitchMovement_ < movementThreshold_ &&
      yawMovement_ < movementThreshold_)
    {
      pointCloudPublisher_.publish(*msg);
    }
  }
}  // namespace pandora_common

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_movement_filter");
  pandora_common::KinectMovementFilter kinectMovementFilter;
  ros::spin();
}
