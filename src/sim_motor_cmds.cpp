#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <fstream>

// This node is a simulation for sending motor commands given a path (vector of coordinates) using turtlesim
// pub: /turtle1/cmd_vel
// sub: /turtle1/pose

// To run this node:
// 1. rosrun turtlesim turtlesim_node
// 2. rosrun sim-motor-cmds sim-motor-cmds_node <path.txt>

// NOTE: the motion controls assume points are always in fron of robot. i.e. 
//       the turning angle is between -pi/2 to pi/2

int signum_double(double val){
    return (0 < val) - (val < 0);
}



class Robot{
	private:

        std::vector<geometry_msgs::Point> path;

        ros::Publisher robot_pub;

		geometry_msgs::Point current_pos; 
        double current_theta = 0;
        double kAngle_threshold;
        size_t path_size;
        size_t path_index = 0; // to keep track the current target

        
    public:

        Robot(ros::NodeHandle& nh, ros::Publisher &pub, std::vector<geometry_msgs::Point> p){

            // turtle is spawned at (5.544, 5.544) by default
            double initial_heading_x = nh.param("initial_x", 5.544);;
            double initial_heading_y = nh.param("initial_y", 5.544);;
            double initial_heading_z = nh.param("initial_z", 0);
            
            // tuning for - overshoot vs. steady state error
            kAngle_threshold = nh.param("angle_threshold", M_PI/256);
            robot_pub = pub;
            path = p;
            path_size = path.size();

            // ROS_INFO_STREAM("path size is: ");
            // ROS_INFO_STREAM(path_size);
            // kLinear_magnitude = nh.param("linear_magnitude", 10);
        } // Robot()



        // update pose then genearte next /cmd_vel
        void callback(const turtlesim::Pose& msg) {

            current_pos.x = msg.x; // should be before while loop
            current_pos.y = msg.y;
            current_theta = msg.theta;

            ROS_INFO_STREAM("updating pose");
            ROS_INFO_STREAM(current_pos.x);
            ROS_INFO_STREAM(current_pos.y);
            ROS_INFO_STREAM(current_theta);


            // path planning here? (not good, prolly should be another callback) update the vector
            // rate of updating pose == rate of A* ??
            // path_index = 0

            geometry_msgs::Point target = get_next_target();

            approach_target(target);
        } // callback()



        void approach_target(geometry_msgs::Point &t) {

            ROS_INFO_STREAM("current target: ");
            ROS_INFO_STREAM(t.x);
            ROS_INFO_STREAM(t.y);

            // use robot pose as center reference
            double delta_x = t.x - current_pos.x;
            double delta_y = t.y - current_pos.y;
            
            // two vectors:
            // robot - <current_pos.x, current_pos.y>
            // target - <delta_x, delta_y>

            // IMPORTANT: atan(y/x) works only when robot aligns with positive x axis
            // So rotate both vectors by current_theta together to make alignment
            // this makes current_theta become 0 (pos x axis)
            double shift = -1 * current_theta;

            // some magic math...
            double temp_x = delta_x * cos(shift) - delta_y * sin(shift);
            double temp_y = delta_x * sin(shift) + delta_y * cos(shift);

            delta_x = temp_x;
            delta_y = temp_y;

            // ROS_INFO_STREAM("dx, dy");
            // ROS_INFO_STREAM(delta_x);
            // ROS_INFO_STREAM(delta_y);

            // atan > 0 -- turn left, atan < 0 -- turn right
            double ang_to_target = atan(delta_y / delta_x);


            // turn first to align before moving forward
            if(fabs(ang_to_target) > kAngle_threshold) {

                geometry_msgs::Twist t;
                t.angular.z = signum_double(ang_to_target);
                robot_pub.publish(t);

            } // if
            else {

                robot_pub.publish(geometry_msgs::Twist{});

                // if x is at target, so is y - same assumption as before
                if(fabs(delta_x) > 0.01) {
                    
                    geometry_msgs::Twist t;
                    // atan > 0 -- turn left, atan < 0 -- turn right
                    t.linear.x = 1 * signum_double(delta_x);
                    robot_pub.publish(t);
                }
                else { // default stall
                    robot_pub.publish(geometry_msgs::Twist{});
                }

            } // else

            
        } // approach_target()

        bool target_ahead(geometry_msgs::Point &t) {

            // only check x, not robust but theoretically works
            double delta_x = t.x - current_pos.x;
            // ROS_INFO_STREAM("delta x");
            // ROS_INFO_STREAM(delta_x);
            return fabs(delta_x) > 0.01;

        } // target_ahead()

        geometry_msgs::Point get_next_target() {

            while(path_index < path_size) {

                if(target_ahead(path.at(path_index))) {
                    break;
                }
                else {
                    if(++path_index == path_size) {

                        robot_pub.publish(geometry_msgs::Twist{});
                        ROS_INFO_STREAM("reached all targets");

                        ros::shutdown();
                        exit(0);
                    }
                }

            } // while()

            return path.at(path_index);
        } // get_next_target()

        
};



int main(int argc, char** argv){

    ros::init(argc, argv, "sim_motor_cmds");
    ros::NodeHandle nh_pub;
    //ros::NodeHandle nh_priv("~");


    std::ifstream in_file(argv[1]);
    if(!in_file.is_open()) {
        ROS_INFO_STREAM("error opening file");
        return 0;
    }

    std::vector<geometry_msgs::Point> fake_path;
    double x, y;

    while(in_file >> x >> y) {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        fake_path.push_back(p);
    }

    ros::Publisher pub = nh_pub.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    Robot my_bot{nh_pub, pub, fake_path};
    ros::Subscriber sub = nh_pub.subscribe("turtle1/pose", 1000, &Robot::callback, &my_bot);

    ros::spin();

    return 0;
}