#include <ros/ros.h>
#include "erc_supervisor/util.h"
#include <erc_supervisor/ErcSupervisor.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace erc
{
    // Typedefs to disambiguate subscriber callbacks
    typedef void (ErcSupervisor::*CurrentGoalConstPtrCb)(const geometry_msgs::PoseStampedConstPtr &);
    typedef void (ErcSupervisor::*CurrentGoalCb)(const geometry_msgs::PoseStamped &);

    typedef void (ErcSupervisor::*StatusCb)(const std_msgs::UInt8ConstPtr &);
    typedef void (ErcSupervisor::*UpdateDetectionThresholdCb)(const std_msgs::UInt16ConstPtr &);

    typedef void (ErcSupervisor::*UpdatePoseSubCb)(const std_msgs::Bool &);
    
    
    static std::string StatusToString(ErcSupervisor::Status status)
    {
        std::string ret{};
        switch(status)
        {
        case ErcSupervisor::Status::Start:
        {
            ret = "Start";
        }
        break;
        case ErcSupervisor::Status::Go:
        {
            ret = "Go";
        }
        break;
        case ErcSupervisor::Status::Pause:
        {
            ret = "Pause";
        }
        break;
        case ErcSupervisor::Status::Success:
        {
            ret = "Success";
        }
        break;
        case ErcSupervisor::Status::SeekOrientation:
        {
            ret = "SeekOrientation";
        }
        break;
        case ErcSupervisor::Status::SeekTranslation:
        {
            ret = "SeekTranslation";
        }
        break;
        case ErcSupervisor::Status::UserRotate:
        {
            ret = "UserRotate";
        }
        break;
        default:
        {
            ret = "Undefined";
        } 
        break;
        }

        return ret;
    }
    
    static geometry_msgs::PoseStamped GetPosedStamped(double x, double y)
    {
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = "map";
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.orientation.w = 1;
        return msg;
    }

    static std::map<int, ErcSupervisor::GoalsQueue> GetGoalsQueue()
    {
        std::map<int, ErcSupervisor::GoalsQueue> ret{};
        ErcSupervisor::GoalsQueue queue1{std::vector<geometry_msgs::PoseStamped>{
            GetPosedStamped(7, 3),
            GetPosedStamped(12.19, 8.73),
        }};
        ErcSupervisor::GoalsQueue queue2{std::vector<geometry_msgs::PoseStamped>{
            GetPosedStamped(18.01, 6.00),
            GetPosedStamped(25.04, 4.36),
        }};
        ErcSupervisor::GoalsQueue queue3{std::vector<geometry_msgs::PoseStamped>{
            GetPosedStamped(29.0 , 4.0 ),
            GetPosedStamped(28.62 , -6.17),
        }};
        ErcSupervisor::GoalsQueue queue4{std::vector<geometry_msgs::PoseStamped>{
            GetPosedStamped(19.26, -6.9),
            GetPosedStamped(16.53, -13.0),
            GetPosedStamped(11.63 , -16.85),
        }};
        ErcSupervisor::GoalsQueue queue5{std::vector<geometry_msgs::PoseStamped>{
            GetPosedStamped(20.80, -15.52),
            GetPosedStamped(27.48, -13.65),
        }};

        ret[1] = queue1;
        ret[2] = queue2;
        ret[3] = queue3;
        ret[4] = queue4;
        ret[5] = queue5;

        return ret;
    } 

    ErcSupervisor::ErcSupervisor()
    {
        nh_ = std::make_unique<ros::NodeHandle>();
        pnh_ = std::make_unique<ros::NodeHandle>("~");


        geometry_msgs::PoseStamped current_goal_ = CurrentPoseGoal();

        InitPublishers(pnh_);
        InitSubscribers(pnh_);
        InitMoveBase(pnh_);
        InitServicesClients(pnh_);
        InitServicesServers(pnh_);
        InitRotateMessages();

        buffer_ = std::make_unique<tf2_ros::Buffer>();
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);

        goals = GetGoalsQueue(); //TODO: !

        last_detection_point_ = std::chrono::steady_clock::now();

        map_frame_ = pnh_->param("map_frame", std::string{"map"});
        ROS_INFO_STREAM("Can transform: " << buffer_->canTransform("base_link", map_frame_, ros::Time(0)));

    }

    void ErcSupervisor::InitRotateMessages()
    {        
        reset_twist_msg.linear.x = 0;
        reset_twist_msg.linear.y = 0;
        reset_twist_msg.linear.z = 0;
        reset_twist_msg.angular.x = 0;
        reset_twist_msg.angular.y = 0;
        reset_twist_msg.angular.z = 0;

        rotate_twist_msg.linear.x = 0;
        rotate_twist_msg.linear.y = 0;
        rotate_twist_msg.linear.z = 0;
        rotate_twist_msg.angular.x = 0;
        rotate_twist_msg.angular.y = 0;
        rotate_twist_msg.angular.z = (3.1415/ 8);
    }

    void ErcSupervisor::InitMoveBase(std::unique_ptr<ros::NodeHandle> & pnh)
    {
        std::string move_base_action = pnh->param("move_base_action",
                                                   std::string{"move_base"});
        move_base_client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(move_base_action, 
        true);

        while (!move_base_client_->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO_STREAM("Waiting for the move_base action server to come up");
        }
    }

    void ErcSupervisor::ClearCostMap()
    {
        std_srvs::Empty srv;
        clear_costmap_client_.call(srv);
    }

    void ErcSupervisor::ResetCostMap()
    {
        std_srvs::Empty srv;
        reset_costmap_client_.call(srv);
    }

    void ErcSupervisor::ResumeCostMap()
    {
        std_srvs::Empty srv;
        resume_costmap_client_.call(srv);
    }

    void ErcSupervisor::InitServicesClients(std::unique_ptr<ros::NodeHandle> & pnh)
    {
        update_pose_from_translation_client_ = pnh->serviceClient<fiducial_slam::UpdatePoseFromTrans>("/update_pose_from_translation");
        update_pose_from_rotation_client_ = pnh->serviceClient<fiducial_slam::UpdatePoseFromRot>("/update_pose_from_rotation");

        clear_costmap_client_ = pnh->serviceClient<std_srvs::Empty>("/clear_costmap");
        reset_costmap_client_ = pnh->serviceClient<std_srvs::Empty>("/reset_costmap");
        resume_costmap_client_ = pnh->serviceClient<std_srvs::Empty>("/resume_costmap");

        rotate_action_client_ = std::make_unique<actionlib::SimpleActionClient<rotator::RotationAction>>("/follow_rotation", 
                            true);
        rotate_action_client_->waitForServer();
    }

    void ErcSupervisor::InitPublishers(std::unique_ptr<ros::NodeHandle> & pnh)
    {
        status_pub_ = pnh->advertise<std_msgs::UInt8>("current_status", 10);
        current_goal_pub_ = pnh->advertise<geometry_msgs::PoseStamped>("current_goal", 1, true);
        current_detection_threshold_ = pnh->advertise<std_msgs::UInt8>("/current_detecion_threshold", 1, true);
        controller_cmd_vel_pub_ = pnh->advertise<geometry_msgs::Twist>("/controllers/diff_drive/cmd_vel", 1, true);
    }

    void ErcSupervisor::InitSubscribers(std::unique_ptr<ros::NodeHandle> & pnh)
    {
        std::string goal_sub_topic = pnh->param("goal_sub_topic", std::string{"goal"});
        goal_sub_ = pnh->subscribe(goal_sub_topic, 1, static_cast<CurrentGoalConstPtrCb>(&ErcSupervisor::CurrentGoal), this);
        status_sub_ = pnh->subscribe("status", 1, static_cast<StatusCb>(&ErcSupervisor::SetStatus), this);
        update_pose_sub_ = pnh->subscribe("/new_pose", 1, static_cast<UpdatePoseSubCb>(&ErcSupervisor::ArTagDetected), this);
        update_threshold_sub_ = pnh->subscribe("/new_threshold", 1, static_cast<UpdateDetectionThresholdCb>(&ErcSupervisor::SetDetectionThreshold), this);
    }

    void ErcSupervisor::InitServicesServers(std::unique_ptr<ros::NodeHandle> & pnh)
    {
        seek_orientation_srv  = pnh->advertiseService("/seek_orientation",&ErcSupervisor::CallbackSeekOrientation, this);
        user_rotate_srv_ = pnh->advertiseService("/user_rotate", &ErcSupervisor::CallbackUserRotate, this);   
        set_goals_srv_ = pnh->advertiseService("/set_goals_id", &ErcSupervisor::CallbackSetGoalsId, this);
    }

    void ErcSupervisor::CurrentGoal(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        CurrentGoal(*msg);
    }

    void ErcSupervisor::Update()
    {
        switch (status_)
        {
            case Status::Go:
            {
                auto move_base_status = move_base_client_->getState();
                
                // ROS_INFO_STREAM("MoveBase Status: " << MoveBaseStateToString(move_base_status.state_));

                switch (move_base_status.state_)
                {
                    case move_base_status.SUCCEEDED:
                    {
                        ROS_INFO_STREAM("MB SUCCEEDED");
                        if (current_queue != -1)
                        {
                            auto& queue = goals[current_queue];
                            if (queue.HasNext())
                            {
                                ROS_INFO_STREAM("popping next goal");
                                CurrentGoal(queue.GetNext());
                                StartGoal();
                            }
                            else
                            {
                                ROS_INFO_STREAM("no more goals, seeking for translation");
                                LaunchSeekTranslation();
                            }
                        }
                        else
                        {
                            ROS_INFO_STREAM("no current queue");

                            LaunchSeekTranslation();
                        }
                    }
                    break;

                    case move_base_status.REJECTED:
                    case move_base_status.ABORTED:
                    case move_base_status.LOST:
                    {
                        if(is_aborted_)
                        {
                            ResetCostMap();
                            is_aborted_ = false;
                        }
                        else
                        {
                            ClearCostMap(); 
                            is_aborted_ = true;
                        }

                        StartGoal();
                    }
                    break;
                    
                    default:
                        break;

                    // TODO handle failure of move base aciton
                    // small distance - turn off costmap
                    // big distance - clear costmap
                    // then send goal again
                    // turn on costmap again


                }
            break;
            }
            case Status::SeekTranslation:
            case Status::SeekOrientation:
            case Status::UserRotate:
            {
                SendRotateMsg();
            }
            break;
        }
        current_goal_pub_.publish(current_goal_);
        std_msgs::UInt8 msg;
        msg.data = static_cast<uint8_t>(status_);
        status_pub_.publish(msg);
        std_msgs::UInt8 threshold_msg;
        threshold_msg.data = detection_threshold_;
        current_detection_threshold_.publish(threshold_msg);
    }

    double ErcSupervisor::DistanceFromGoal()
    {
        geometry_msgs::TransformStamped current_pose;
        current_pose = buffer_->lookupTransform(map_frame_, "base_link", ros::Time(0));

        double x = current_pose.transform.translation.x - current_goal_.pose.position.x;
        double y = current_pose.transform.translation.y - current_goal_.pose.position.y;
        double distance = sqrt(x*x + y*y);

        return distance;
    }

    void ErcSupervisor::ArTagDetected(const std_msgs::Bool &msg)
    {
        switch(status_)
        {
            case Status::Go: 
            case Status::Pause: 
            case Status::Success:
            {
                auto new_detection_point = std::chrono::steady_clock::now();
                auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(new_detection_point - last_detection_point_).count();
                    
                if (time_diff >= detection_threshold_)
                {
                    Stop();
                    fiducial_slam::UpdatePoseFromRot request;
                    auto result = update_pose_from_rotation_client_.call(request);
                    last_detection_point_ = std::chrono::steady_clock::now();
                    if (status_ == Status::Go)
                    {
                        StartGoal();
                    }
                }
                else
                {
                    ROS_INFO_STREAM("Time elapsed since last detection less than threshold");
                }
            }
            break;
            case Status::SeekTranslation:
            {
                ROS_INFO_STREAM("Seek Translation");
                Stop();
                
                fiducial_slam::UpdatePoseFromRot srv;
                update_pose_from_rotation_client_.call(srv);
             
                if (srv.response.success)
                {   
                    double distance = DistanceFromGoal();
                    ROS_INFO_STREAM("distance form goal: " << distance);
                    if (distance < min_distance_from_goal_)
                    {
                        status_ = Status::Success;
                    }
                    else 
                    {
                        StartGoal();
                        status_ = Status::Go;

                    }
                }

                else
                {
                    ROS_INFO_STREAM("found no tags");
                    status_ = Status::Success;
                }
                rotation_timer.stop();
            }
            break;

            case Status::SeekOrientation:
            {
                Stop();
                fiducial_slam::UpdatePoseFromTrans srv;
                srv.request.translation = trans_from_user_;

                update_pose_from_translation_client_.call(srv);

                if (!srv.response.success)
                {
                    ROS_ERROR("orientation not updated");
                }

                status_ = Status::Start;
                rotation_timer.stop();
            }
            break;

            default:
                break;
            
        }

    }
    
    void ErcSupervisor::SetDetectionThreshold(const std_msgs::UInt16ConstPtr &msg)
    {
        detection_threshold_ = msg->data;
    }

    void ErcSupervisor::CurrentGoal(const geometry_msgs::PoseStamped &msg)
    {
        ROS_INFO_STREAM("Received new goal message. Cancelling current route.");
        Stop();
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
    
    void ErcSupervisor::RotateTimerCallback(const ros::TimerEvent& event)
    {
        ROS_INFO_STREAM("timer calback");
        switch (status_)
        {
            case Status::SeekOrientation:
            {
                ROS_INFO_STREAM("Timeout when trying to find new orientation");
                Stop();
                status_ = Status::Start;
            }
            break;
            
            case Status::SeekTranslation:
            {
                ROS_INFO_STREAM("Timeout when trying to find new translation");
                Stop();
                status_ = Status::Success;
            }
            break;
            
            default:
            {
                ROS_INFO_STREAM("Timeout is no longer relevant");
            }
            break;
        }
    }

    void ErcSupervisor::StartGoal()
    {
        // goal_pub_.publish(current_goal_);
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose = current_goal_; 
        goal.target_pose.header.stamp = ros::Time::now() - ros::Duration(0.1);

        move_base_client_->sendGoal(goal);
    }


    void ErcSupervisor::Reset()
    {
        Stop();
        current_goal_ = CurrentPoseGoal();
        is_aborted_ = false;
        current_queue = -1;
        for (auto &el : goals)
        {
            el.second.Reset();
        }
        ROS_INFO_STREAM("Reseted goals");
        ResumeCostMap();
    }

    void ErcSupervisor::Stop()
    {
        move_base_client_->cancelAllGoals();
        StopRotateMsg();
        ros::Duration(1.5).sleep();
        ClearCostMap();
    }

     
    void ErcSupervisor::StopRotateMsg()
    {
        controller_cmd_vel_pub_.publish(reset_twist_msg);
    } 

    void ErcSupervisor::SendRotateMsg()
    {
        controller_cmd_vel_pub_.publish(rotate_twist_msg);
    }

    bool ErcSupervisor::CallbackSeekOrientation(erc_supervisor::SeekOrientation::Request & req, erc_supervisor::SeekOrientation::Response &res)
    {
        trans_from_user_ = req.position;
        Rotate(ros::Duration(req.timeout), Status::SeekOrientation);
    }

    bool ErcSupervisor::CallbackSetGoalsId(erc_supervisor::SetGoalsId::Request & req,  erc_supervisor::SetGoalsId::Response &res)
    {
        int id = -1;
        if(status_ == Status::Start)
        {
            if(req.id >= goals.size())
            {
                res.result = false;
                res.status_msg = "Id greater than map size";
            }
            else
            {
                auto find = goals.find(req.id);
                
                if(find!=goals.end())
                {
                    res.result = true;
                    res.status_msg = "New queue id";
                    CurrentGoal(find->second.goals[0]);
                    id = req.id;
                }


            }
        }
        else
        {
            res.result = false;
            res.status_msg = "Set Start mode firstly";
        }

        current_queue = id;
    }

    bool ErcSupervisor::CallbackUserRotate(erc_supervisor::UserRotate::Request & req,
                                           erc_supervisor::UserRotate::Response &res)
    {
        if (status_ == Status::Pause || 
            status_ == Status::Start ||
            status_ == Status::Success)
        {
            Rotate(ros::Duration(req.timeout), Status::UserRotate, req.degree);
            res.success = true;
        }
        else
        {
            res.success = false;
        }
        return res.success;
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
        case Status::Start:
        {
            ROS_INFO_STREAM("Start status received, stopping rover and cancelling goal");
            Reset();
            status_ = new_status;
            break;
        }
        break;
        case Status::Go:
        {
            switch (status_)
            {
                case Status::Go:
                case Status::Pause:
                case Status::Start:
                {
                    ROS_INFO_STREAM("Go status received, sending move_base goal.");
                    StartGoal();
                    status_ = new_status;
                }
                break;
                default:
                {
                    ROS_INFO_STREAM("Current status: " << static_cast<int>(status_) << "doesn't support transition to Go");
                }
                break;
            }
        }
        break;
        case Status::Pause:
        {
            switch (status_)
            {
            case Status::Go:
            case Status::SeekTranslation:
            case Status::SeekOrientation:
            case Status::UserRotate:
            {
                ROS_INFO_STREAM("Pause status received, cancelling move_base goal, but leaving goal unchanged for resumption.");
                Stop();
                status_ = new_status;
                break;
            }
            default:
                ROS_INFO_STREAM("Current status" <<  StatusToString(status_) << "doesn't support transition to Pause");
                break;
            }
        }
        break;
        default:
            // ROS_INFO_STREAM("Status %d is not supported", new_status);
            break;
        }
        
    }

    void ErcSupervisor::Rotate(ros::Duration timeout, ErcSupervisor::Status status, double degrees)
    {
        if (status == Status::SeekTranslation || status == Status::SeekOrientation)
        {
            rotation_sign_ = -rotation_sign_;
            ROS_INFO_STREAM("SEEK status");
            timer = pnh_->createTimer(timeout, &ErcSupervisor::RotateTimerCallback, this, true);
            ROS_INFO_STREAM("created timer");
        }
        else 
        {
            ROS_INFO_STREAM("Custom rotate received. Timeout: " << timeout.toSec() << ", Angle: " << degrees);
            LaunchRotateAction(degrees, timeout); 
        }

        status_ = status;        
    }

    void ErcSupervisor::LaunchRotateAction(double degree, ros::Duration duration)
    {
        rotator::RotationGoal goal;
        goal.angle = degree;
        goal.timeout = duration.toSec();

        rotate_action_client_->sendGoal(goal, 
            boost::bind(&ErcSupervisor::UserRotateDoneCallback, this, _1,_2), 
                                RotationClient::SimpleActiveCallback(), 
                                RotationClient::SimpleFeedbackCallback());
    }

    void ErcSupervisor::UserRotateDoneCallback(const actionlib::SimpleClientGoalState& state,
        const rotator::RotationResultConstPtr& result)
    {
        Stop();
        status_ = Status::Pause;
    }

    void ErcSupervisor::LaunchSeekTranslation()
    {
        Rotate(ros::Duration(20.0, 0), Status::SeekTranslation);
    }

    void ErcSupervisor::LaunchSeekOrientation()
    {
        Rotate(ros::Duration(20.0, 0), Status::SeekOrientation);
    }

    void ErcSupervisor::SetStatus(const std_msgs::UInt8ConstPtr &msg)
    {
        Status new_status = static_cast<Status>(msg->data);
        SetStatus(new_status);
    }

} // namespace erc