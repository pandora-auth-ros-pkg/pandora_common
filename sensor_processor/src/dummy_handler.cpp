#include "sensor_processor/dummy_handler.h"
#include "sensor_processor/dummy_preprocessor.h"
#include "sensor_processor/dummy_processor.h"
#include "sensor_processor/dummy_postprocessor.h"


namespace sensor_processor
{
  DummyHandler::
  DummyHandler(const std::string& ns) : Handler<Int32, Int32, Int32, Int32>(ns)
  {
  }

  void
  DummyHandler::
  startTransition(int newState)
  {
    currentState_ = newState;

    switch (currentState_)  // ................
    {
      case 2:
        preProcPtr_.reset( new DummyPreProcessor("~/preprocessor", this) );
        processorPtr_.reset( new DummyProcessor("~/processor", this) );
        postProcPtr_.reset( new DummyPostProcessor("~/postprocessor", this) );
        break;
      case state_manager_msgs::RobotModeMsg::MODE_TERMINATING:
        ros::shutdown();
        return;
    }
    previousState_ = currentState_;
    transitionComplete(currentState_);
  }

  void
  DummyHandler::
  completeTransition()
  {
  }
}  // namespace sensor_processor
