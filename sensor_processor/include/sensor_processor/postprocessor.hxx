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

#include "sensor_processor/postprocessor.h"

namespace sensor_processor
{
  template <class VisionOutput, class PublishedType>
  PostProcessor<VisionOutput, PublishedType>::PostProcessor(NodeHandlePtr nhPtr)
  {
    nhPtr_ = nhPtr;
    getTopicName();
    publisher_ = nhPtr_->advertise<PublishedType>(publisherTopic_, 1);
    
    ROS_INFO_NAMED(PKG_NAME, "[PostProcessor] Initialized");
  }
  
  template <class VisionOutput, class PublishedType>
  PostProcessor<VisionOutput, PublishedType>::~PostProcessor()
  {
  }
  
  template <class VisionOutput, class PublishedType>
  void PostProcessor<VisionOutput, PublishedType>::getTopicName()
  {
    std::string ns = nhPtr_->getNamespace();

    if (nhPtr_->getParam(ns + "/published_topic", publisherTopic_))
    {
      publisherTopic_ = ns + "/" + publisherTopic_;
      ROS_INFO_NAMED(PKG_NAME, "[PostProcessor] Published to topic");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME, "[PostProcessor] Could not find topic to publish to");
    }
  }
  
  template <class VisionOutput, class PublishedType>
  void PostProcessor<VisionOutput, PublishedType>::setVisionOutput(const VisionOutput& input)
  {
    output_ = input;
  }
  
  template <class VisionOutput, class PublishedType>
  void PostProcessor<VisionOutput, PublishedType>::getPublisherResult(const PublishedTypePtr& result)
  {
    result.reset(&publishedType_);
    publisher_.publish(publishedType_);
  }
}  // namespace sensor_processor
