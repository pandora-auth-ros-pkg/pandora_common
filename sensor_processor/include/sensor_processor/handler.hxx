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
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef SENSOR_PROCESSOR_HANDLER_HXX
#define SENSOR_PROCESSOR_HANDLER_HXX

#include <string>
#include <boost/algorithm/string.hpp>

#include "sensor_processor/ProcessorLogInfo.h"
#include "sensor_processor/processor.h"
#include "sensor_processor/preprocessor.h"
#include "sensor_processor/postprocessor.h"
#include "sensor_processor/handler.h"
#include "sensor_processor/processor_error.h"

namespace sensor_processor
{
  template <class SubType, class ProcInput, class ProcOutput, class PubType>
    Handler<SubType, ProcInput, ProcOutput, PubType>::
    Handler(const std::string& ns)
    {
      nhPtr_.reset( new ros::NodeHandle(ns) );
      name_ = boost::to_upper_copy(ros::this_node::getName());
      currentState_ = state_manager_msgs::RobotModeMsg::MODE_OFF;
      previousState_ = state_manager_msgs::RobotModeMsg::MODE_OFF;

      processorInputPtr_.reset( new ProcInput() );
      processorOutputPtr_.reset( new ProcOutput() );
      processorResultPtr_.reset( new PubType() );

      nhPtr_->param<std::string>("op_report_topic", reportTopicName_,
          ros::this_node::getName() + "/processor_log");
      operationReport_ = nhPtr_->advertise<ProcessorLogInfo>(
          reportTopicName_, 10);

      if (!nhPtr_->getParam("subscribed_topic", inputTopic_))
      {
        ROS_FATAL("subscribed_topic param not found");
        ROS_BREAK();
      }
      nSubscriber_ = nhPtr_->subscribe(inputTopic_, 1,
          &Handler::completeProcessCallback, this);

      if (!nhPtr_->getParam("published_topic", outputTopic_))
      {
        ROS_FATAL("published_topic param not found");
        ROS_BREAK();
      }
      nPublisher_ = nhPtr_->advertise<PubType>(outputTopic_, 1);

      clientInitialize();
      ROS_INFO("[%s] Handler initialized", name_.c_str());
    }

  template <class SubType, class ProcInput, class ProcOutput, class PubType>
    Handler<SubType, ProcInput, ProcOutput, PubType>::
    ~Handler()
    {
      ROS_INFO("[%s] Handler terminated", name_.c_str());
    }

  template <class SubType, class ProcInput, class ProcOutput, class PubType>
    ros::NodeHandlePtr
    Handler<SubType, ProcInput, ProcOutput, PubType>::shareNodeHandle()
    {
      return nhPtr_;
    }

  template <class SubType, class ProcInput, class ProcOutput, class PubType>
    void
    Handler<SubType, ProcInput, ProcOutput, PubType>::
    completeProcessCallback(
        const boost::shared_ptr<SubType const>& subscribedTypePtr)
    {
      ROS_INFO("Received msg!");
      bool success = true;  //!< checker for success of operations

      // First a preprocessing operation happens
      boost::shared_ptr< PreProcessor<SubType, ProcInput> > preProcPtr(
          boost::dynamic_pointer_cast< PreProcessor<SubType, ProcInput> >(preProcPtr_));
      preProcPtr->setInputPtr(subscribedTypePtr);
      preProcPtr->setOutputPtr(processorInputPtr_);
      try {
        success = preProcPtr->process();
      }
      catch (processor_error& e) {
        completeProcessFinish(false, e.what());
        return;
      }
      if (!success) {
        completeProcessFinish(success, "PreProcessor");
        return;
      }

      boost::shared_ptr< Processor< ProcInput, ProcOutput> > processorPtr(
          boost::dynamic_pointer_cast< Processor<ProcInput, ProcOutput> >(processorPtr_));
      processorPtr->setInputPtr(processorInputPtr_);
      processorPtr->setOutputPtr(processorOutputPtr_);
      try {
        success = processorPtr->process();
      }
      catch (processor_error& e) {
        completeProcessFinish(false, e.what());
        return;
      }
      if (!success) {
        completeProcessFinish(success, "Processor");
        return;
      }

      boost::shared_ptr< PostProcessor< ProcOutput, PubType> > postProcPtr(
          boost::dynamic_pointer_cast< PostProcessor<ProcOutput, PubType> >(postProcPtr_));
      postProcPtr->setInputPtr(processorOutputPtr_);
      postProcPtr->setOutputPtr(processorResultPtr_);
      try {
        success = postProcPtr->process();
      }
      catch (processor_error& e) {
        completeProcessFinish(false, e.what());
        return;
      }
      if (success)
        nPublisher_.publish(*processorResultPtr_);
      completeProcessFinish(success, "Finished");
    }

  template <class SubType, class ProcInput, class ProcOutput, class PubType>
    void
    Handler<SubType, ProcInput, ProcOutput, PubType>::
    completeProcessFinish(bool success, const std::string& logInfo)
    {
      ProcessorLogInfo processorLogInfo;
      processorLogInfo.success = success;
      processorLogInfo.logInfo = logInfo;
      operationReport_.publish(processorLogInfo);
    }
}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_HANDLER_HXX
