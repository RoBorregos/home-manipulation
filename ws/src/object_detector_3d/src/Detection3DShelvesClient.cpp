// Test for the shelve placing service
#include <iostream>

#include <ros/ros.h>

#include <object_detector_3d/ShelvePlacePositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <vector>

enum class ShelvePlacePositionState
{
    EXCLUDED,
    TARGET_HEIGHT,
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shelve_place_position_client");

    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<object_detector_3d::ShelvePlacePositionAction> as_("detect3d_place_shelves", true);

    ROS_INFO("Waiting for action server to start.");
    as_.waitForServer();

    ROS_INFO("Action server started, sending goal.");

    object_detector_3d::ShelvePlacePositionGoal goal;

    ShelvePlacePositionState state = ShelvePlacePositionState::TARGET_HEIGHT;
    bool ignore_moveit_ = false;
    std::vector<double> excluded_heights{0.78, 1.44};
    float target_height = -1;
    int n_objects_ = 0;
    
    if (state == ShelvePlacePositionState::TARGET_HEIGHT)
    {
       target_height = 0.78;
       n_objects_ = 3;
    }

    goal.ignore_moveit = ignore_moveit_;
    goal.excluded_heights = excluded_heights;
    goal.target_height = target_height;
    goal.n_objects = n_objects_;

    as_.sendGoal(goal);

    bool finished_before_timeout = as_.waitForResult(ros::Duration(60.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = as_.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        as_.cancelGoal();
    }


    return 0;
}

