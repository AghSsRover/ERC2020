#ifndef AGH_SS_UTIL_H_
#define AGH_SS_UTIL_H_

#include <string>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace erc
{
    std::string MoveBaseStateToString(actionlib::SimpleClientGoalState::StateEnum state)
    {
        using MBState = actionlib::SimpleClientGoalState::StateEnum;
        std::string ret{};

        switch(state)
        {
            case MBState::PENDING:
            {
                ret = "PENDING";
            }
            break;
            case MBState::ACTIVE:
            {
                ret = "ACTIVE";
            }
            break;
            case MBState::RECALLED:
            {
                ret = "RECALLED";
            }
            break;
            case MBState::REJECTED:
            {
                ret = "REJECTED";
            }
            break;
            case MBState::PREEMPTED:
            {
                ret = "PREEMPTED";
            }
            break;
            case MBState::ABORTED:
            {
                ret = "ABORTED";
            }
            break;
            case MBState::SUCCEEDED:
            {
                ret = "SUCCEEDED";
            }
            break;
            case MBState::LOST:
            {
                ret = "LOST";
            }
            break;
        }
        return ret;
    }
}

#endif