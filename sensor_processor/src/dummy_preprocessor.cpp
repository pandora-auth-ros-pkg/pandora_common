#include "sensor_processor/dummy_preprocessor.h"

namespace sensor_processor
{
  DummyPreProcessor::
  DummyPreProcessor(const std::string& ns, AbstractHandler* handler) :
  PreProcessor<std_msgs::Int32, std_msgs::Int32>(ns, handler)
  {
    data_ = 2;
  }

  bool
  DummyPreProcessor::
  preProcess(const std_msgs::Int32ConstPtr& input, const std_msgs::Int32Ptr& output)
  {
    ROS_INFO("pre processor: %d", input->data);
    output->data = data_;
    return true;
  }
}  // namespace sensor_processor
