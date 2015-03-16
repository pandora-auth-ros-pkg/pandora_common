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
* 
*********************************************************************/

#include "sensor_processor/handler.h"

namespace sensor_processor
{
  template <class SubscribedType, class VisionInput, class VisionOutput, class PublishedType>
  Handler<SubscibedType, VisionInput, VisionOutput, PublishedType>::Handler(): 
    preProcessor_(nhPtr_(new ros::NodeHandle("")), &Handler<SubscibedType, VisionInput, VisionOutput, 
    PublishedType>::completeProcessCallback(const SubscribedTypePtr& subscribedTypePtr)), 
    postProcessor_(nhPtr_)
  {
    ROS_INFO_NAMED(PKG_NAME, "[Handler] Initialized");
  }
  
  template <class SubscribedType, class VisionInput, class VisionOutput, class PublishedType>
  Handler<SubscibedType, VisionInput, VisionOutput, PublishedType>::~Handler()
  {
    ROS_INFO_NAMED(PKG_NAME, "[Handler] Terminated");
  }
  
  template <class SubscribedType, class VisionInput, class VisionOutput, class PublishedType>
  void Handler<SubscibedType, VisionInput, VisionOutput, PublishedType>::completeProcessCallback(const 
    SubscribedTypePtr& subscribedTypePtr)
  {
    subscribedType_ = *subscibedTypePtr;
    
    preProcessor_.setSubscriberInput(&subscribedType_);
    preProcessor_.preProcess();
    VisionInputPtr visionInput(new VisionInput);
    preProcessor_.getVisionResult(visionInput);
    
    processor_.setInput(*visionInput);
    processor_.process();
    VisionOutputPtr visionOutput(new VisionOutput);
    processor_.getResult(visionOutput);
    
    postProcessor_.setVisionOutput(*visionOutput);
    postProcessor_.postProcess();
    postProcessor_.getPublisherResult(&publishedType_);
  }
  
  template <class SubscribedType, class VisionInput, class VisionOutput, class PublishedType>
  void Handler<SubscibedType, VisionInput, VisionOutput, PublishedType>::startTransition(int newState)
  {
    
  }
  
  template <class SubscribedType, class VisionInput, class VisionOutput, class PublishedType>
  void Handler<SubscibedType, VisionInput, VisionOutput, PublishedType>::completeTransition()
  {
    ROS_INFO("[Sensor Processor] : Transition Complete");
  }
}  // namespace sensor_processor
