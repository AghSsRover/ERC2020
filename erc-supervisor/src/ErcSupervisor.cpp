#include <ros/ros.h>

#include <erc_supervisor/ErcSupervisor.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace erc
{
    // Typedefs to disambiguate subscriber callbacks
    typedef void (ErcSupervisor::*CurrentGoalConstPtrCb)(const geometry_msgs::PoseStampedConstPtr &);
    typedef void (ErcSupervisor::*CurrentGoalCb)(const geometry_msgs::PoseStamped &);

    typedef void (ErcSupervisor::*StatusCb)(const std_msgs::UInt8ConstPtr &);

    typedef void (ErcSupervisor::*UpdatePoseSubCb)(const std_msgs::Bool &);


    ErcSupervisor::ErcSupervisor()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        std::string goal_sub_topic = pnh.param("goal_sub_topic", std::string{"goal"});
        goal_sub_ = pnh.subscribe(goal_sub_topic, 1, static_cast<CurrentGoalConstPtrCb>(&ErcSupervisor::CurrentGoal), this);

        // No point to have this configurable. Deal with it boiii.
        // std::string status_sub_topic = pnh.param("status_sub_topic", std::string{"status"});
        status_sub_ = pnh.subscribe("status", 1, static_cast<StatusCb>(&ErcSupervisor::SetStatus), this);

        std::string move_base_action = pnh.param("move_base_action", std::string{"move_base"});
        move_base_client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(move_base_action, true);
       
        while (!move_base_client_->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        
        update_pose_sub_ = pnh.subscribe("/new_pose", 1, static_cast<UpdatePoseSubCb>(&ErcSupervisor::ArTagDetected), this);
        
        update_pose_client_ = pnh.serviceClient<std_srvs::Empty>("/update_pose");

        map_frame_ = pnh.param("map_frame", std::string{"map"});

        buffer_ = std::make_unique<tf2_ros::Buffer>();
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);

        current_goal_ = CurrentPoseGoal();

        status_pub_ = pnh.advertise<std_msgs::UInt8>("current_status", 10);

        current_goal_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("current_goal", 1, true);

        ROS_INFO_STREAM("Can transform: " << buffer_->canTransform("base_link", map_frame_, ros::Time(0)));
    }

    ErcSupervisor::~ErcSupervisor() {}

    void ErcSupervisor::CurrentGoal(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        CurrentGoal(*msg);
    }
    
    void ErcSupervisor::ArTagDetected(const std_msgs::Bool& msg)
    {
        if(msg.data)
        {
            SetStatus(Status::Pause);
            std_srvs::Empty request;
            update_pose_client_.call(request);
            SetStatus(Status::Go);
        }
    }

    void ErcSupervisor::CurrentGoal(const geometry_msgs::PoseStamped &msg)
    {
        ROS_INFO_STREAM("Received new goal message. Cancelling current route.");
        move_base_client_->cancelAllGoals();
        try
        {
            current_goal_ = buffer_->transform(msg, map_frame_, ros::Duration(1.0));
            current_goal_.header.frame_id = map_frame_;
            ROS_INFO_STREAM("Goal accepted and transformed from \'" << msg.header.frame_id << "\' to \'" << map_frame_ << "\' frame.");
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("Failed to accept new goal: " << e.what() << "\nA goal at the robot's current position will be saved instead.");
            current_goal_ = CurrentPoseGoal();
        }
    }

    void ErcSupervisor::StartGoal()
    {
        // goal_pub_.publish(current_goal_);
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose = current_goal_;
        goal.target_pose.header.stamp = ros::Time::now();

        move_base_client_->sendGoal(goal);
    }

    void ErcSupervisor::PauseGoal()
    {
        move_base_client_->cancelAllGoals();
    }

    void ErcSupervisor::Reset()
    {
        move_base_client_->cancelAllGoals();
        current_goal_ = CurrentPoseGoal();
    }

    ErcSupervisor::Status ErcSupervisor::CurrentStatus()
    {
        return status_;
    }

    geometry_msgs::PoseStamped ErcSupervisor::CurrentPoseGoal()
    {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "base_link";
        goal.pose.orientation.w = 1.0;

        return goal;
    }

    void ErcSupervisor::SetStatus(Status new_status)
    {
        if (new_status == status_)
        {
            return;
        }

        switch (new_status)
        {
        case Status::Go:
            ROS_INFO_STREAM("\'Go\' status received, sending move_base goal.");
            StartGoal();
            break;
        case Status::Pause:
            ROS_INFO_STREAM("\'Pause\' status received, cancelling move_base goal, but leaving goal unchanged for resumption.");
            PauseGoal();
            break;
        case Status::Reset:
            ROS_INFO_STREAM("\'Reset\' status received, cancelling move_base goal, and resetting goal to current base_link pose.");
            Reset();
            break;
        default:
            ROS_INFO_STREAM("Unsupported status \'" << static_cast<int>(new_status) << "\' received.");
            break;
        }
    }

    void ErcSupervisor::SetStatus(const std_msgs::UInt8ConstPtr &msg)
    {
        Status new_status = static_cast<Status>(msg->data);

        SetStatus(new_status);
    }

    void ErcSupervisor::Update()
    {
        auto move_base_status = move_base_client_->getState();
        Status new_status;

        if (move_base_status.isDone())
        {
            status_ = Status::Idle;
        }
        else
        {
            status_ = Status::Go;
        }

        current_goal_pub_.publish(current_goal_);
        std_msgs::UInt8 msg;
        msg.data = static_cast<uint8_t>(status_);
        status_pub_.publish(msg);

        // switch (move_base_status.state_)
        // {
        //     case move_base_status.ACTIVE:
        //     case move_base_status.PENDING:
        //         new_status = Status::Go;
        //         break;
        //     case move_base_status.PREEMPTED:
        //     case move_base_status.RECALLED:
        //     case move_base_status.REJECTED:
        //     case move_base_status.ABORTED:
        //         new_status = Status::Pause;
        //         break;
        //     case move_base_status.SUCCEEDED:
        //         new_status = Status::Idle;
        //         break;
        // }
    }

} // namespace erc