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

#include "sensor_processor/vision_postprocessor.h"

namespace sensor_processor
{
  template <class PublishedMessageType>
  VisionPostProcessor<PublishedMessageType>::VisionPostProcessor(NodeHandlePtr nhPtr, StringPtr frameId): 
    PostProcessor(nhPtr), frameId_(frameId), nodeFrameTimestamp_(time)
  {
  }
  
  template <class PublishedMessageType>
  VisionPostProcessor<PublishedMessageType>::~VisionPostProcessor()
  {
  }
  
  template <class PublishedMessageType>
  template<class Type> void VisionPostProcessor<PublishedMessageType>::getParameter(const std::string& name, 
    const Type& param)
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
  
  template <class PublishedMessageType>
  bool VisionPostProcessor<PublishedMessageType>::getParentFrameId()
  {
    
  }
  
  template <class PublishedMessageType>
  void VisionPostProcessor<PublishedMessageType>::getGeneralParameters()
  {
    getParameter<int>("image_width", frameWidth_);
    getParameter<int>("image_height", frameHeight_);
    getParameter<double>("hfov", hfov_);
    getParameter<double>("vfov", vfov_);
    // .........
  }
  
  template <class PublishedMessageType>
  void VisionPostProcessor<PublishedMessageType>::findAnglesOfRotation()
  {
    //~ for (int ii = 0; ii < output_.size(); i++)
    //~ {
      //~ float x = output_[i].x - static_cast<float>(frameWidth_) / 2;
      //~ float y = static_cast<float>(frameHeight_) / 2 - output_[i].y;
      //~ 
      //~ yaw_[i] = atan(2 * x / frameWidth_ * tan(hfov / 2));
      //~ pitch_[i] = atan(2 * y / frameHeight_ * tan(vfov / 2));
    //~ }
    
    
  }
}  // namespace sensor_processor
