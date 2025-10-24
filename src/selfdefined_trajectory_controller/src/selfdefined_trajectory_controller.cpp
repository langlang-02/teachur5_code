#include "selfdefined_trajectory_controller/selfdefined_trajectory_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include <trajectory_interface/quintic_spline_segment.h>

namespace position_controllers
{
typedef selfdefined_controllers::SelfDefinedTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                            hardware_interface::PositionJointInterface>
    SelfDefinedTrajectoryController;
}
namespace velocity_controllers
{
typedef selfdefined_controllers::SelfDefinedTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                            hardware_interface::VelocityJointInterface>
    SelfDefinedTrajectoryController;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::SelfDefinedTrajectoryController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(velocity_controllers::SelfDefinedTrajectoryController, controller_interface::ControllerBase)
