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

#ifndef SENSOR_PROCESSOR_VISION_POSTPROCESSOR_H
#define SENSOR_PROCESSOR_VISION_POSTPROCESSOR_H

#include "sensor_processor/postprocessor.h"

namespace sensor_processor
{
  template <class PublishedMessageType>
  class VisionPostProcessor: public PostProcessor<std::vector<cv::Point>, PublishedMessageType>
  {
    public:
      typedef boost::shared_ptr<std::string> StringPtr;
      typedef boost::shared_ptr<ros::Time> TimePtr;
      
      VisionPostProcessor(NodeHandlePtr nhPtr, StringPtr frameId, TimePtr time);
      virtual ~VisionPostProcessor();

      void findAnglesOfRotation(); 
      
    protected:
      const StringPtr frameId_;
      const TimePtr nodeFrameTimestamp_;
      
      //
      double hfov_;  // caps?
      double vfov_;
      
      int frameWidth_;
      int frameHeight_;
      int cameraIndicator_;
      
      std::string cameraName_;
      std::string parentFrameId_;
      
      bool getParentFrameId();
      void getGeneralParameters();
      template<class Type> void getParameter(const std::string& name, const Type& param);
      //
  };
}  // namespace sensor_processor
#endif  // SENSOR_PROCESSOR_VISION_POSTPROCESSOR_H
