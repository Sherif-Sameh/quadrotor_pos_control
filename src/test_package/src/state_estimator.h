#ifndef STATE_ESTIMATOR
#define STATE_ESTIMATOR

#include <cmath>
#include <vector>
#include <string>
#include <boost/circular_buffer.hpp>
#include "spline.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "test_package/FullState.h"

using std::vector;
using std::string;
using boost::circular_buffer;

class StateEstimator
{
public:
    /* Constructor, Destructor and Public Methods */
    StateEstimator(ros::NodeHandle& nh);
    void callback_uav_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);


private:
    /* Private Methods */
    bool applySlidingWindow();
    void updateVelocityEstimates(vector<double>& velocity_estimate);
    void publishMessages(const geometry_msgs::Pose& pose, const vector<double>& velocity_estimate);

    /* Class Members */

    // Input parameters - configuration:
    string pose_topic_name;
    size_t pose_history_size;
    double pose_sampling_rate;
    vector<double> fir_filter_coefficients;

    // FIR filter related (circular buffers for O(1) insertion without shifting):
    circular_buffer<double> pose_x_raw;
    circular_buffer<double> pose_y_raw;
    circular_buffer<double> pose_z_raw;

    // Filtered pose histroy related (to use for spline fitting or numerical differentiation):
    vector<double> pose_x_filtered;
    vector<double> pose_y_filtered;
    vector<double> pose_z_filtered;
    vector<double> pose_time;

    // ROS related:
    ros::Publisher state_inertial_pub;
    ros::Subscriber pose_sub;
};

#endif