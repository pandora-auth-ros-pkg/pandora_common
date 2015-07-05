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

#include <string>
#include <map>
#include <vector>

#include <ros/ros.h>
#include <ros/forwards.h>
#include <pluginlib/class_loader.h>

#include "state_manager_msgs/RobotModeMsg.h"

#include "sensor_processor/handler.h"
#include "sensor_processor/abstract_processor.h"

#ifndef SENSOR_PROCESSOR_DYNAMIC_HANDLER_H
#define SENSOR_PROCESSOR_DYNAMIC_HANDLER_H

namespace sensor_processor
{
  /**
   * @class DynamicHandler Class which enables plugin loading/unloading of
   * abstract processors
   */
  class DynamicHandler : public Handler
  {
   public:
    static int ROBOT_STATES(const std::string& X) {
      static std::map<std::string, int>* m = NULL;
      if (m == NULL)
      {
        m = new std::map<std::string, int>;
        m->insert(std::make_pair("OFF", 0));
        m->insert(std::make_pair("START_AUTONOMOUS", 1));
        m->insert(std::make_pair("EXPLORATION_RESCUE", 2));
        m->insert(std::make_pair("IDENTIFICATION", 3));
        m->insert(std::make_pair("SENSOR_HOLD", 4));
        m->insert(std::make_pair("SEMI_AUTONOMOUS", 5));
        m->insert(std::make_pair("TELEOPERATED_LOCOMOTION", 6));
        m->insert(std::make_pair("SENSOR_TEST", 7));
        m->insert(std::make_pair("EXPLORATION_MAPPING", 8));
        m->insert(std::make_pair("TERMINATING", 9));
      }
      return (*m)[X];
    }

    static std::string ROBOT_STATES(int X) {
      static std::map<int, std::string>* m = NULL;
      if (m == NULL)
      {
        m = new std::map<int, std::string>;
        m->insert(std::make_pair(0, "OFF"));
        m->insert(std::make_pair(1, "START_AUTONOMOUS"));
        m->insert(std::make_pair(2, "EXPLORATION_RESCUE"));
        m->insert(std::make_pair(3, "IDENTIFICATION"));
        m->insert(std::make_pair(4, "SENSOR_HOLD"));
        m->insert(std::make_pair(5, "SEMI_AUTONOMOUS"));
        m->insert(std::make_pair(6, "TELEOPERATED_LOCOMOTION"));
        m->insert(std::make_pair(7, "SENSOR_TEST"));
        m->insert(std::make_pair(8, "EXPLORATION_MAPPING"));
        m->insert(std::make_pair(9, "TERMINATING"));
      }
      return (*m)[X];
    }

   public:
    explicit DynamicHandler(bool load=true);

    template <class PreProcessor>
    void
    loadPreProcessor(const std::string& processor_name);
    void
    loadPreProcessor(const std::string& processor_name, const std::string& processor_type);
    void
    checkAndLoadPreProcessor(const std::string& processor_name, const std::string& processor_type);

    template <class Processor>
    void
    loadProcessor(const std::string& processor_name);
    void
    loadProcessor(const std::string& processor_name, const std::string& processor_type);
    void
    checkAndLoadProcessor(const std::string& processor_name, const std::string& processor_type);

    template <class PostProcessor>
    void
    loadPostProcessor(const std::string& processor_name);
    void
    loadPostProcessor(const std::string& processor_name, const std::string& processor_type);
    void
    checkAndLoadPostProcessor(const std::string& processor_name, const std::string& processor_type);

   protected:
    /**
      * @brief Function that performs all the needed procedures when the robot's
      * state is changed
      * @param newState [int] Robot's new state
      */
    virtual void
    startTransition(int newState);

    /**
      * @brief Function that is called after the transition from one state to
      * another is completed
      */
    virtual void
    completeTransition();

    /**
      * @brief Load Post Processor implementation class if available
      * @param name [const std::string&] name of implementation class
      */
    void
    loadProcessor(AbstractProcessorPtr& processorPtr,
                  const std::string& processor_name, const std::string& processor_type);

   protected:
    //!< States in which node is active
    std::vector<std::string> activeStates_;

   private:
    //!< Plugin PostProcessor loader
    pluginlib::ClassLoader<AbstractProcessor> processor_loader_;
    //!< Map containing nodes to be loaded dynamically with state change
    std::map< int, boost::array<std::string, 3> > state_to_processor_map_;

    std::string previousPreProcessorType_;
    std::string previousProcessorType_;
    std::string previousPostProcessorType_;
  };
}  // namespace sensor_processor

#include "sensor_processor/dynamic_handler.hxx"

#endif  // SENSOR_PROCESSOR_DYNAMIC_HANDLER_H
