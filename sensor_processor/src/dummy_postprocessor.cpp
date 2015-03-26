#include "sensor_processor/dummy_postprocessor.h"

namespace sensor_processor
{
  DummyPostProcessor::
  DummyPostProcessor(const std::string& ns, AbstractHandler* handler) :
  PostProcessor<std_msgs::Int32, std_msgs::Int32>(ns, handler)
  {
    data_ = 4;
  }

  bool
  DummyPostProcessor::
  postProcess(const std_msgs::Int32ConstPtr& input, const std_msgs::Int32Ptr& output)
  {
    ROS_INFO("post processor: %d", input->data);
    output->data = data_;
    return true;
  }
}  // namespace sensor_processor
