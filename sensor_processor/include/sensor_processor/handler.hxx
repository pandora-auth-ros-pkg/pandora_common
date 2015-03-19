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

#include "sensor_processor/handler.h"

namespace sensor_processor
{
  template <class SubscribedType, class ProcessorInput, class ProcessorOutput, class PublishedType>
  Handler<SubscibedType, ProcessorInput, ProcessorOutput, PublishedType>::Handler()
  {
    currentState_ = state_manager_msgs::RobotModeMsg::MODE_OFF;
    previousState_ = state_manager_msgs::RobotModeMsg::MODE_OFF;

    processorInputPtr_.reset( new ProcessorInput() );
    processorOutputPtr_.reset( new ProcessorOutput() );

    clientInitialize();
    nodeNowOn_ = false;
    ROS_INFO_NAMED(PKG_NAME, "[Handler] Initialized");
  }

  template <class SubscribedType, class ProcessorInput, class ProcessorOutput, class PublishedType>
  Handler<SubscibedType, ProcessorInput, ProcessorOutput, PublishedType>::~Handler()
  {
    ROS_INFO_NAMED(PKG_NAME, "[Handler] Terminated");
  }

  template <class SubscribedType, class ProcessorInput, class ProcessorOutput, class PublishedType>
  void Handler<SubscribedType, ProcessorInput, ProcessorOutput, PublishedType>::
  completeProcessCallback(const SubscribedTypePtr& subscribedTypePtr)
  {
    preProcessorPtr_->setSubscriberInput(subscribedTypePtr);
    preProcessorPtr_->process();
    preProcessorPtr_->getProcessorInput(processorInputPtr_);

    processorPtr_->setInput(processorInputPtr_);
    processorPtr_->process();
    processorPtr_->getResult(processorOutputPtr_);

    postProcessorPtr_->setSubscriberInput(subscribedTypePtr);
    postProcessorPtr_->setProcessorOutput(processorOutputPtr_);
    postProcessorPtr_->process();
  }
}  // namespace sensor_processor
