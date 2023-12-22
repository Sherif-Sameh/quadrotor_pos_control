#include "state_estimator.h"

/* Constructor, Destructor and Public Methods */

StateEstimator::StateEstimator(ros::NodeHandle& nh)
{
    // Receive Input Parameters from the Parameter Server:
    nh.param("/state_estimator/pose_topic_name", pose_topic_name, string("/drone_pose"));
    int pose_history_size_temp;
    nh.param("/state_estimator/pose_history_size", pose_history_size_temp, 10);
    pose_history_size = static_cast<size_t>(pose_history_size_temp);
    nh.param("/state_estimator/pose_sampling_rate", pose_sampling_rate, 50.0);
    nh.param("/state_estimator/fir_filter_coefficients", fir_filter_coefficients, vector<double>(5, 0.2));

    // Report values of Input Parameters:
    ROS_INFO("Pose topic name: %s", pose_topic_name.c_str());
    ROS_INFO("Pose history size: %zu", pose_history_size);
    ROS_INFO("Pose sampling rate (Hz): %.2f", pose_sampling_rate);
    ROS_INFO("FIR filter frame size: %zu", fir_filter_coefficients.size());
    ROS_INFO("FIR filter first and last coefficients: %.5f and %.5f", fir_filter_coefficients.front(), fir_filter_coefficients.back());

    // Initialize any needed variables:
    pose_x_raw.set_capacity(fir_filter_coefficients.size());
    pose_y_raw.set_capacity(fir_filter_coefficients.size());
    pose_z_raw.set_capacity(fir_filter_coefficients.size());
    pose_x_filtered.reserve(pose_history_size);
    pose_y_filtered.reserve(pose_history_size);
    pose_z_filtered.reserve(pose_history_size);
    pose_time.resize(pose_history_size);
    for(size_t i = 0; i < pose_history_size; i++)
    {
        pose_time[i] = i / pose_sampling_rate;
    }

    // Initializing ROS publishers, subscribers, service servers and service clients:
    state_inertial_pub  = nh.advertise<test_package::FullState>("/state_estimator/fullstate", 1, true);
    pose_sub            = nh.subscribe(pose_topic_name, 2, &StateEstimator::callback_uav_pose, this);        
}

void StateEstimator::callback_uav_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Add new elements to the pose circular buffers:
    pose_x_raw.push_back(msg->pose.position.x);
    pose_y_raw.push_back(msg->pose.position.y);
    pose_z_raw.push_back(msg->pose.position.z);

    // If the buffers are full then start filtering:
    if(pose_x_raw.size() == fir_filter_coefficients.size())
    {
        // Apply sliding window and update filtered pose:
        bool pose_filtered_vector_full = applySlidingWindow();

        // If we have a full filtered pose vector then start estimating velocities:
        if(pose_filtered_vector_full)
        {
            vector<double> velocity_estimate(3, 0.0);
            updateVelocityEstimates(velocity_estimate);

            // Publish full state message:
            publishMessages(msg->pose, velocity_estimate);
        }
    }
}

/* Private Methods */

bool StateEstimator::applySlidingWindow()
{
    vector<double> position_filtered(3, 0.0);
    size_t window_size = fir_filter_coefficients.size();

    // Apply the FIR filter window to all 3 axes simultaneously:
    for(size_t i = 0; i < window_size; i++)
    {
        position_filtered[0] += pose_x_raw[i] * fir_filter_coefficients[i];
        position_filtered[1] += pose_y_raw[i] * fir_filter_coefficients[i];
        position_filtered[2] += pose_z_raw[i] * fir_filter_coefficients[i];
    }

    // Update corresponding pose vectors:
    bool pose_vector_full = false;
    if(pose_x_filtered.size() == pose_history_size)
    {
        // Erase the first element and shift all elements to the left:
        pose_vector_full = true;
        pose_x_filtered.erase(pose_x_filtered.begin());
        pose_y_filtered.erase(pose_y_filtered.begin());
        pose_z_filtered.erase(pose_z_filtered.begin());
    }
    pose_x_filtered.push_back(position_filtered[0]);
    pose_y_filtered.push_back(position_filtered[1]);
    pose_z_filtered.push_back(position_filtered[2]);

    return pose_vector_full;
}

void StateEstimator::updateVelocityEstimates(vector<double>& velocity_estimate)
{
    // Fit the cubic splines to the filtered pose vectors parameterized by time:
    tk::spline pose_x_spline(pose_time, pose_x_filtered, tk::spline::cspline, false,
                                        tk::spline::second_deriv, 0.0, tk::spline::second_deriv, 0.0);
    tk::spline pose_y_spline(pose_time, pose_y_filtered, tk::spline::cspline, false,
                                        tk::spline::second_deriv, 0.0, tk::spline::second_deriv, 0.0);
    tk::spline pose_z_spline(pose_time, pose_z_filtered, tk::spline::cspline, false,
                                        tk::spline::second_deriv, 0.0, tk::spline::second_deriv, 0.0);
    
    // Evaluate the magnitude of the velocity at the last point and update estimates:
    double pose_time_last = pose_time.back();
    velocity_estimate[0] = pose_x_spline.deriv(1, pose_time_last);
    velocity_estimate[1] = pose_y_spline.deriv(1, pose_time_last);
    velocity_estimate[2] = pose_z_spline.deriv(1, pose_time_last);
}

void StateEstimator::publishMessages(const geometry_msgs::Pose& pose, const vector<double>& velocity_estimate)
{
    // Declaring any messages to be published: 
    test_package::FullState full_state_msg;
    
    // Filling in the message parameters:
    full_state_msg.header.stamp = ros::Time::now();
    full_state_msg.pose = pose;
    full_state_msg.twist.linear.x = velocity_estimate[0];
    full_state_msg.twist.linear.y = velocity_estimate[1];
    full_state_msg.twist.linear.z = velocity_estimate[2];
    
    // Publishing all messages:
    state_inertial_pub.publish(full_state_msg);
}

/* End of class definition */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_estimator");
    ros::NodeHandle nh;

    StateEstimator state_estimator(nh);
    ros::spin();

    return 0;
}