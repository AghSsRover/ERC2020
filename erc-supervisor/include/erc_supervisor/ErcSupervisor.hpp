#ifndef AGH_SS_ERC_SUPERVISOR_HPP
#define AGH_SS_ERC_SUPERVISOR_HPP

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>

#include <std_srvs/Empty.h>

#include <move_base_msgs/MoveBaseAction.h>
// #include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
// #include <actionlib_msgs/GoalID.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <memory>
#include <chrono>
namespace erc
{

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

        virtual ~ErcSupervisor();

        /**
         * @brief Current supervisor status.
         * 
         */
        enum class Status
        {
            Idle,  // Not navigating anywhere, no goal set. This is the default state.
            Go,    // Currently navigating to a goal. Also used to command the robot to start navigating to the goal.
            Pause, // Transient state: Pause navigation, but remember goal position remembered. Switch to Idle state, can resume with 'Go'
            Reset  // Transient state: Abandon route, switch to Idle state
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
        void Reset();

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

    private:


        geometry_msgs::PoseStamped CurrentPoseGoal();

        /**
         * @brief Callback for status messages, forwards them to the SetStatus logic.
         * 
         * @param msg 
         */
        void SetStatus(const std_msgs::UInt8ConstPtr& msg);

        /**
         * @brief Callback for settings time threshold for detection.
         * 
         * @param msg 
         */

        void SetDetectionThreshold(const std_msgs::UInt16ConstPtr& msg);

        /**
         * @brief Callback for AR Tag detection signal.
         * 
         * @param msg 
         */

        void ArTagDetected(const std_msgs::Bool& msg);

        /**
         * @brief Current status.
         * 
         * This is currently not used, as the supervisor is stateless.
         */
        Status status_ = Status::Idle;

        /**
         * @brief Current goal.
         * Resets to [0, 0, 0] (robot's current pose) when in Idle state.
         */
        geometry_msgs::PoseStamped current_goal_;

        std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client_;

        /**
         * @brief Subscriber for status commands.
         * 
         * Status commands can be used to control the behavior of the supervisor.
         * Specifically, a "Go", "Pause" or "Reset" command can be sent to
         * control the interactions with move_base.
         */
        ros::Subscriber status_sub_;

        /**
         * @brief Subscriber for the current goal.
         * 
         */
        ros::Subscriber goal_sub_;

        /**
         * @brief Subscriber for the information about net AR Tag detection.
         * 
         */
        ros::Subscriber update_pose_sub_;
         
        /**
         * @brief Subscribes threshold for Ar Tag detection.
         */

        ros::Subscriber update_threshold_sub_;
         
        /**
         * @brief Publishes the supervisor's current status as an uint8.
         */
        ros::Publisher status_pub_;

        /**
         * @brief Invokes request for updating position.
         */
        ros::ServiceClient update_pose_client_;

        /**
         * @brief Publishes the supervisor's currently saved goal, for visualization and debugging.
         */
        ros::Publisher current_goal_pub_;

         /**
         * @brief Publishes the supervisor's currently detection threshold/
         */
        ros::Publisher current_detection_thrshold_;


        std::string map_frame_;

        std::unique_ptr<tf2_ros::Buffer> buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        std::chrono::time_point<std::chrono::steady_clock> last_detection_point_;
        uint16_t detection_threshold_ = 15; // in seconds
        // tf2_ros::Buffer buffer_;
        // tf2_ros::TransformListener tf_listener_;

    };


} // namespace erc

#endif // AGH_SS_ERC_SUPERVISOR_HPP