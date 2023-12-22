#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/GetModelState.h>

using std::string;

class ModelPosePublisher
{
public:
    /* Constructor and public methods */
    ModelPosePublisher(ros::NodeHandle &nh)
    {
        // Retrieve the input parameters from the parameter server
        nh.param("/model_pose_publisher/model_label", model_label, string("model"));
        nh.param("/model_pose_publisher/publish_rate", publish_rate, 25.0);

        // Report values of input parameters
        ROS_INFO("Model label: %s", model_label.c_str());
        ROS_INFO("Publish rate: %.4f", publish_rate);

        // Set topic and service names
        string pose_topic = string("/") + model_label + string("/pose");
        string twist_topic = string("/") + model_label + string("/twist");

        // Initialize any publishers, subscribers, clients or servers
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1, true);
        twist_pub = nh.advertise<geometry_msgs::TwistStamped>(twist_topic, 1, true);
        get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    }

    void spin()
    {
        ros::Rate loop_rate(publish_rate);

        while(ros::ok())
        {
            // Call service to get latest pose
            gazebo_msgs::GetModelState get_model_state_srv;
            get_model_state_srv.request.model_name = model_label;
            get_model_state_srv.request.relative_entity_name = string("world");
            
            // If call is successful then publish latest pose
            if(get_model_state_client.call(get_model_state_srv))
            {
                publishPose(get_model_state_srv.response.header, get_model_state_srv.response.pose, get_model_state_srv.response.twist);
            }

            // Sleep till next iteration
            loop_rate.sleep();
        }
    }

private:
    /* Private methods */
    void publishPose(const std_msgs::Header &header, const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = header;
        pose_msg.pose = pose;
        pose_pub.publish(pose_msg);

        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header = header;
        twist_msg.twist = twist;
        twist_pub.publish(twist_msg);
    }

    /* Class members */
    string model_label;
    double publish_rate;

    ros::ServiceClient get_model_state_client;
    ros::Publisher pose_pub;
    ros::Publisher twist_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model_pose_publisher");
    ros::NodeHandle nh;

    ModelPosePublisher model_pose_publisher(nh);
    model_pose_publisher.spin();

    return 0;
}