/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
#include "sensor_processor/handler.h"
#include "sensor_processor/processor_error.h"

namespace sensor_processor
{

  Handler::
  Handler() {}

  void
  Handler::
  onInit()
  {
    state_manager::StateClientNodelet::onInit();

    nh_ = this->getPublicNh();
    private_nh_ = this->getPrivateNh();
    name_ = this->getNodeName();

    std::string reportTopicName;

    private_nh_.param<std::string>("op_report_topic", reportTopicName, "processor_log");
    operation_report_ = private_nh_.advertise<ProcessorLogInfo>(reportTopicName, 1);

    clientInitialize();
    ROS_INFO("[%s] initialized", name_.c_str());
  }

  Handler::
  ~Handler()
  {
    ROS_INFO("[%s] terminated", name_.c_str());
  }

  template <class SubType>
  void Handler::completeProcessCallback(
      const boost::shared_ptr<SubType const>& subscribedTypePtr)
  {
    bool success = true;  //!< checker for success of operations
    boost::shared_ptr<boost::any> subTypePtr( new boost::any(subscribedTypePtr) );
    boost::shared_ptr<boost::any> processorInputPtr( new boost::any );
    boost::shared_ptr<boost::any> processorOutputPtr( new boost::any );
    boost::shared_ptr<boost::any> processorResultPtr( new boost::any );

    // First a preprocessing operation happens
    try {
      success = preProcPtr_->process(subTypePtr, processorInputPtr);
    }
    catch (processor_error& e) {
      completeProcessFinish(false, e.what());
      return;
    }
    if (!success) {
      completeProcessFinish(success, "pre_processor");
      return;
    }

    try {
      success = processorPtr_->process(processorInputPtr, processorOutputPtr);
    }
    catch (processor_error& e) {
      completeProcessFinish(false, e.what());
      return;
    }
    if (!success) {
      completeProcessFinish(success, "processor");
      return;
    }

    try {
      success = postProcPtr_->process(processorOutputPtr, processorResultPtr);
    }
    catch (processor_error& e) {
      completeProcessFinish(false, e.what());
      return;
    }
    completeProcessFinish(success, "finished");
  }

  void Handler::completeProcessFinish(bool success, const std::string& logInfo)
  {
    ProcessorLogInfoPtr processorLogInfo( new ProcessorLogInfo );
    processorLogInfo->success = success;
    processorLogInfo->logInfo = logInfo;
    operation_report_.publish(processorLogInfo);
  }
}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_HANDLER_HXX
