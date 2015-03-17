/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Authors:
* Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
*********************************************************************/

#include "sensor_processor/vision_preprocessor.h"

namespace sensor_processor
{
  VisionPreProcessor::VisionPreProcessor(NodeHandlePtr nhPtr, 
    void (*callback)(const SubscribedTypePtr& subscribedTypePtr)): PreProcessor(nhPtr, callback)
  {
    parentFrameId_ = "";
    frameId_ = "";
    
    //getParam...............?
    
    ROS_INFO_NAMED(PKG_NAME, "[VisionPreProcessor] Initialized");  // for each vision node..............
  }
  
  VisionPreProcessor::~VisionPreProcessor()
  {
  }
  
  template<class Type>
  void VisionPreProcessor::getParameter(const std::string& name, const Type& param)
  {
    if (nh_.getParam(name, param))
    {
      ROS_DEBUG_STREAM(name << " : " << param);
    }
    else
    {
      ROS_FATAL("[Node] : Parameter " << name << " not found. Using Default");
      ROS_BREAK();
    }
  }

  bool VisionPreProcessor::getParentFrameId()
  {
    
  }
  
  void VisionPreProcessor::preProcess()
  {
    cv_bridge::CvImagePtr inMsg;
    inMsg = cv_bridge::toCvCopy(subscribedType_, sensor_msgs::image_encodings::BGR8);
    input_ = inMsg->image.clone();
    nodeFrameTimestamp_ = subscribedType_.header.stamp;
    frameId_ = subscribedType_.header.frame_id;
    frameWidth_ = subscribedType_.width;  // uint32
    frameHeight_ = subscribedType_.height;
    
    if (frameId_[0] == '/')  // ??????????
    {
      frameId_ = frameId_.substr(1);
      cameraIndicator_ = 1;
    }
    
    if (input_.empty())
    {
      ROS_ERROR("[Node] No more Frames or something went wrong with bag file");
      // ros::shutdown();
      return;
    }
  }
}  // namespace sensor_processor
