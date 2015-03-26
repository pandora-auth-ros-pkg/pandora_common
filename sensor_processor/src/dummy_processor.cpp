#include "sensor_processor/dummy_processor.h"

namespace sensor_processor
{
  DummyProcessor::
  DummyProcessor(const std::string& ns, AbstractHandler* handler) :
  Processor<std_msgs::Int32, std_msgs::Int32>(ns, handler)
  {
    data_ = 3;
  }

  bool
  DummyProcessor::
  process(const std_msgs::Int32ConstPtr& input, const std_msgs::Int32Ptr& output)
  {
    ROS_INFO("processor: %d", input->data);
    output->data = data_;
    return true;
  }
}  // namespace sensor_processor
