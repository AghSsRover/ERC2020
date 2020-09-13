#ifndef AGH_SS_ERC_SUPERVISOR_HPP
#define AGH_SS_ERC_SUPERVISOR_HPP

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>

#include <std_srvs/Empty.h>

#include <move_base_msgs/MoveBaseAction.h>
// #include <geometry_msgs/Pose.h>
// #include <actionlib_msgs/GoalID.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <memory>
#include <chrono>


#include <fiducial_slam/UpdatePoseFromRot.h>
#include <fiducial_slam/UpdatePoseFromTrans.h>

#include <erc_supervisor/SetGoalsId.h>
#include <erc_supervisor/SeekOrientation.h>
#include <erc_supervisor/UserRotate.h>

#include <rotator/RotationAction.h>
namespace erc
{
    typedef actionlib::SimpleActionClient<rotator::RotationAction> RotationClient;

    /**
     * @brief Utility class to help manage autonomous actions.
     * 
     * This class interacts with move_base and AR tag detection packages.
     * It can set a destination for move_base, pause it, and resume it later.
     * 
     * The goal can be set by publishing a PoseStamped message,
     * and the 'Go', 'Pause', 'Resume' and 'Reset' actions can be
     * triggered by publishing the appropriate Status value.
     * 
     * TODO: Automated stops to look for AR Tags.
     * (Maybe the AR Tag node should pause the supervisor, publish the localization corrections, and then resume???)
     */
    class ErcSupervisor
    {
    public:

        

        ErcSupervisor();

        virtual ~ErcSupervisor() = default;

        /**
         * @brief Current supervisor status.
         * 
         */
        enum class Status
        {
            Start,  // Not navigating anywhere, no goal set. This is the default state.
            Go,    // Currently navigating to a goal. Also used to command the robot to start navigating to the goal.
            Pause, // Transient statex`: Pause navigation, but remember goal position remembered. Switch to Idle state, can resume with 'Go'
            Success, // Goal achieved
            SeekOrientation, // Rotating to update orientation of robot
            SeekTranslation, // Rotating  to update translation of robot
            UserRotate // Rotation invoked by user
        };

        struct GoalsQueue
        {
            GoalsQueue() = default;
            GoalsQueue(std::vector<geometry_msgs::PoseStamped>  goals)
            : goals(goals)
            {}

            virtual ~GoalsQueue() = default;

            size_t Size() const noexcept {return goals.size();}
            size_t HasNext() const noexcept {return current +1 < goals.size();}
            geometry_msgs::PoseStamped GetNext() noexcept {auto ret = goals.at(current); current+=1; return ret;}
            void Reset() {current = 0;}

            std::vector<geometry_msgs::PoseStamped> goals;
            size_t current = 0;
        };

        /**
         * @brief Set current goal, which will be sent to move_base when needed.
         * 
         * This also cancels any route currently being executed by move_base.
         * @param msg The pose to save
         */
        void CurrentGoal(const geometry_msgs::PoseStamped &msg);

        /**
         * @overload erc::ErcSupervisor::CurrentGoal(const geometry_msgs::Pose &msg)
         * 
         * @param msg The pose to save
         */
        void CurrentGoal(const geometry_msgs::PoseStampedConstPtr &msg);

        /**
         * @brief Start moving towards the goal.
         * 
         */

        void StartGoal();

        /**
         * @brief Pause navigation towards the goal, but remember it.
         * 
         * 
         * The route can be resumed later.
         */
        void PauseGoal();

        /**
         * @brief Abandon navigation and forget the goal.
         * 
         * This resets the goal to the robot's current pose. ([0, 0, 0] in base_link frame.)
         */


        void RotateTimerCallback(const ros::TimerEvent&);

        void Reset();

        void Stop();

        void UserRotateDoneCallback(const actionlib::SimpleClientGoalState& state,
                                    const rotator::RotationResultConstPtr& result);
    
        
        

        /**
         * @brief Get the current status of the supervisor.
         * 
         * @return Status 
         */
        Status CurrentStatus();

        /**
         * @brief Set the status of the supervisor.
         * 
         * Currently, this supports a Go, Pause and Reset status.
         * @param new_status 
         */
        void SetStatus(Status new_status);

        /**
         * @brief Fetch current move_base status and change the supervisor's state accordingly.
         * 
         */
        void Update();
        geometry_msgs::PoseStamped CurrentPoseGoal();

    private:

        std::unique_ptr<ros::NodeHandle> nh_;
        std::unique_ptr<ros::NodeHandle> pnh_;

        void InitPublishers(std::unique_ptr<ros::NodeHandle> & pnh);
        void InitSubscribers(std::unique_ptr<ros::NodeHandle> & pnh);
        void InitMoveBase(std::unique_ptr<ros::NodeHandle> & pnh);
        void InitServicesClients(std::unique_ptr<ros::NodeHandle> & pnh);
        void InitServicesServers(std::unique_ptr<ros::NodeHandle> & pnh);

        void InitRotateMessages();

        geometry_msgs::PoseStamped current_goal_;

        Status status_ = Status::Start;
        
        ros::Subscriber status_sub_;
        ros::Subscriber goal_sub_;
        ros::Subscriber update_pose_sub_;
        ros::Subscriber update_threshold_sub_;

        ros::Publisher status_pub_;
        ros::Publisher current_goal_pub_;
        ros::Publisher current_detection_threshold_;
        ros::Publisher controller_cmd_vel_pub_;

        // SubsCallbacks
        void SetDetectionThreshold(const std_msgs::UInt16ConstPtr& msg);
        void ArTagDetected(const std_msgs::Bool& msg);
        void SetStatus(const std_msgs::UInt8ConstPtr& msg);

        ros::ServiceClient update_pose_from_rotation_client_;
        ros::ServiceClient update_pose_from_translation_client_;
        ros::ServiceClient clear_costmap_client_;
        ros::ServiceClient reset_costmap_client_;
        ros::ServiceClient resume_costmap_client_;

        bool is_aborted_ = false;

        ros::ServiceServer seek_orientation_srv ;
        ros::ServiceServer user_rotate_srv_;
        ros::ServiceServer set_goals_srv_;

        bool CallbackSeekOrientation(erc_supervisor::SeekOrientation::Request & req, erc_supervisor::SeekOrientation::Response &res);
        bool CallbackUserRotate(erc_supervisor::UserRotate::Request & req,  erc_supervisor::UserRotate::Response &res);
        bool CallbackSetGoalsId(erc_supervisor::SetGoalsId::Request & req,  erc_supervisor::SetGoalsId::Response &res);

        void ClearCostMap();
        void ResetCostMap();
        void ResumeCostMap();
    
        ros::Timer rotation_timer;
        void Rotate(ros::Duration timeout,
                    Status status=Status::UserRotate,
                    double degrees = 0.0);
        void LaunchSeekTranslation();
        void LaunchSeekOrientation();
        void LaunchRotateAction(double degree, ros::Duration duration);
        void StopRotateMsg(); 
        void SendRotateMsg();
        double DistanceFromGoal();

        std::chrono::time_point<std::chrono::steady_clock> last_detection_point_; // TODO ros duration
        uint16_t detection_threshold_ = 15; // in seconds


        std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client_;
    
        std::string map_frame_;

        std::unique_ptr<tf2_ros::Buffer> buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;


        int rotation_sign_ = 1; 
        double min_distance_from_goal_ = 1.0;
        double angular_distance_traveled_;

        geometry_msgs::Twist rotate_twist_msg;
        geometry_msgs::Twist reset_twist_msg;

        geometry_msgs::Vector3 trans_from_user_;

        std::unique_ptr<actionlib::SimpleActionClient<rotator::RotationAction>> rotate_action_client_;

        std::map<int, GoalsQueue> goals;
        int current_queue = -1;

        ros::Timer timer;
    };


} // namespace erc

#endif // AGH_SS_ERC_SUPERVISOR_HPP