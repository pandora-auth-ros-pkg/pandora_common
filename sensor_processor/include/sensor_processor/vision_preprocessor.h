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

#ifndef SENSOR_PROCESSOR_VISION_PREPROCESSOR_H
#define SENSOR_PROCESSOR_VISION_PREPROCESSOR_H

#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"
#include "sensor_processor/preprocessor.h"

namespace sensor_processor
{
  class VisionPreProcessor: public PreProcessor<sensor_msgs::Image, cv::Mat>
  {
    public:
      VisionPreProcessor(NodeHandlePtr nhPtr, void (*callback)(const SubscribedTypePtr& subscribedTypePtr));
      virtual ~VisionPreProcessor();
      
    private:
      double hfov_;
      double vfov_;
      
      int frameWidth_;
      int frameHeight_;
      
      int cameraIndicator_;
      cv::Mat nodeFrame_;
      
      ros::Time nodeFrameTimestamp_;
      
      std::string cameraName_;
      std::string parentFrameId_;
      std::string frameId_;
      
      bool getParentFrameId();
      template<class Type> void getParameter(const std::string& name, const Type& param);
  };
}  // namespace sensor_processor
#endif  // SENSOR_PROCESSOR_VISION_PREPROCESSOR_H
