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
 *********************************************************************/

#ifndef SENSOR_PROCESSOR_DYNAMIC_HANDLER_HXX
#define SENSOR_PROCESSOR_DYNAMIC_HANDLER_HXX

#include <string>
#include <vector>

#include <ros/ros.h>

#include "state_manager_msgs/RobotModeMsg.h"

#include "sensor_processor/dynamic_handler.h"

namespace sensor_processor
{

  DynamicHandler::
  DynamicHandler(bool load)
  {
    if (load)
    {
      int ii, jj;
      nh_ = this->getPublicNodeHandle();
      private_nh_ = this->getPrivateNodeHandle();
      name_ = this->getName();

      activeStates_.clear();
      // Get Active States
      XmlRpc::XmlRpcValue active_states;
      if (!privateNh_.getParam("active_states", active_states))
      {
        ROS_FATAL("[%s] Cound not find active robot states", name_.c_str());
        ROS_BREAK();
      }
      ROS_ASSERT(active_states.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (ii = 0; ii < active_states.size(); ++ii) {
        ROS_ASSERT(active_states[ii].getType() == XmlRpc::XmlRpcValue::TypeString);
        activeStates_.push_back(static_cast<std::string>(active_states[ii]));
      }

      for (ii = 0; ii < activeStates_.size(); ++ii) {
        XmlRpc::XmlRpcValue state_processors;
        if (!private_nh_.getParam("state_processors/"+activeStates_[ii], state_processors))
        {
          ROS_FATAL("[%s] Cound not find processor per state", name_.c_str());
          ROS_BREAK();
        }
        ROS_ASSERT(state_processors.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(state_processors.size() == 3);
        boost::array<std::string, 3> processors_of_state;
        for (jj = 0; jj < 3; ++jj) {
          ROS_ASSERT(state_processors[jj] == XmlRpc::XmlRpcValue::TypeString);
          processors_of_state[jj] = state_processors[jj];
        }
        state_to_processor_map_.insert(std::make_pair(
              ROBOT_STATES(activeStates_[ii]), processors_of_state));
      }
    }
  }

  void
  DynamicHandler::
  startTransition(int newState)
  {
    this->previousState_ = this->currentState_;
    this->currentState_ = newState;

    bool previouslyOff = true;
    bool currentlyOn = false;

    for (int ii = 0; ii < activeStates_.size(); ii++) {
      previouslyOff = (previouslyOff && this->previousState_ != ROBOT_STATES(activeStates_[ii]));
      currentlyOn = (currentlyOn || this->currentState_ == ROBOT_STATES(activeStates_[ii]));
    }

    if (!previouslyOff && !currentlyOn)
    {
      this->preProcPtr_.reset();
      this->processorPtr_.reset();
      this->postProcPtr_.reset();
    }
    else if (previouslyOff && currentlyOn)
    {
      loadProcessor(this->preProcPtr_,
                    "~preprocessor", state_to_processor_map_[this->currentState_][0]);
      loadProcessor(this->processorProcPtr_,
                    "~processor", state_to_processor_map_[this->currentState_][1]);
      loadProcessor(this->postProcPtr_,
                    "~postprocessor", state_to_processor_map_[this->currentState_][2]);
    }
    else
    {
      checkAndLoadProcessor(this->preProcPtr_, 0);
      checkAndLoadProcessor(this->processorPtr_, 1);
      checkAndLoadProcessor(this->postProcPtr_, 2);
    }


    if (this->currentState_ == state_manager_msgs::RobotModeMsg::MODE_TERMINATING)
    {
      this->preProcPtr_.reset();
      this->processorPtr_.reset();
      this->postProcPtr_.reset();

      ROS_INFO("[%s] Terminating", name_.c_str());
      ros::shutdown();
      return;
    }

    transitionComplete(this->currentState_);
  }

  void
  DynamicHandler::
  completeTransition() {}

  void
  DynamicHandler::
  loadProcessor(AbstractProcessorPtr& processorPtr,
      const std::string& processor_name, const std::string& processor_type)
  {
    try
    {
      processorPtr = postProcessorLoader_.createInstance(name);
      processorPtr->initialize(processor_name, this);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("[%s] Failed to create a %s processor, are you sure it is properly"
                " registered and that the containing library is built? "
                "Exception: %s", name_.c_str(), processor_type, ex.what());
      ROS_BREAK();
    }
  }

  void
  DynamicHandler::
  checkAndLoadProcessor(AbstractProcessorPtr& processorPtr,
      const std::string& processor_name, int type)
  {
    if (state_to_processor_map_[this->previousState_][type] ==
        state_to_processor_map_[this->currentState_][type])
      return;
    loadProcessor(processorPtr,
                  processor_name, state_to_processor_map_[this->currentState_][type]);
  }

}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_DYNAMIC_HANDLER_HXX
