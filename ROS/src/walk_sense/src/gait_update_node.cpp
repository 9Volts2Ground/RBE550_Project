#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <sstream>

// Easy logging macro
#define LOG( x ) std::cout << x << std::endl;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Define hardware constants here for now. Need to figure out cleaner way to cooperate
// with rospy and roscpp
const int num_legs = 6;

// Define waling gait constants here for now. ToDO
const float max_stride_length = 0.068f; // meters
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class gait_update_node{

//---------------------------------------------------------
private:
    std::array<float, 6> distance_foot_traveled = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // How far each foot traveled last increment



//---------------------------------------------------------
public:

    //=====================================================
    // Declare public variables
    ros::NodeHandle node; // Initialize the ROS node

    std::array<ros::Publisher, num_legs>  pub_leg; // Each leg gets its own topic, so make an array of publishers

    ros::Rate loop_rate;

    //=====================================================
    // Class constructor
    gait_update_node() : loop_rate(10) {

        // Publishes leg states
        for ( int leg=0; leg < pub_leg.size(); leg++ ){
            std::stringstream leg_state_num;
            leg_state_num << "leg_state" << leg;

            pub_leg[leg] = node.advertise<std_msgs::String>(leg_state_num.str(), 1);
        }
    };

    //=====================================================
    // Clears out distance each foot traveled for next loop around
    void clear_distance_foot_traveled(){
        distance_foot_traveled = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    };

    //=====================================================
    // Function to update gait state
    void update_gait( ){

        // Variable declaration and initialization
        ros::Time t = ros::Time::now(); // Current time
        ros::Time t_previous = t; // Previous time through the loop

        float phase = 0.0f; // Which phase we are in during our stride

        int count = 0;
        while( ros::ok() )
        {

            // Get true time delta from previous calculation, in case node gets gummed up
            t = ros::Time::now();
            ros::Duration dt = t - t_previous;

            // Calculate gait phase
            phase += *std::max_element(std::begin(distance_foot_traveled), std::end(distance_foot_traveled))
                    / max_stride_length;

            // Clear list of distance each foot traveled
            clear_distance_foot_traveled();


            // for ( int leg=0; leg < 6; leg++ ){

            //     std_msgs::String msg;
            //     std::stringstream ss;

            //     ss << "hello leg" << leg << " msg_num: " << count;
            //     msg.data = ss.str();

            //     pub_leg[leg].publish( msg ); // Print out
            // }
            count++;

            t_previous = t;

            // Sleep until the next loop time
            ros::spinOnce();
            loop_rate.sleep();
        }

    };
};

//=============================================================================
int main( int argc, char **argv )
{

    // Initialize ROS
    ros::init( argc, argv, "gait_update_node_cpp" );

    // Initialize your class with publishers and subscribers
    gait_update_node node;

    node.update_gait();

    ros::spin();

    return 0;
}

