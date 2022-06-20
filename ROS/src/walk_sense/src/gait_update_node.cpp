#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#define LOG( x ) std::cout << x << std::endl;

class gait_update_node{


//-------------------------------------
public:

    //=====================================================
    // Declare public variables
    ros::NodeHandle node; // Initialize the ROS node

    std::array<ros::Publisher, 6>  pub_leg; // Each leg gets its own topic, so make an array of publishers

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
    // Function to update gait state
    void update_gait( ){

        int count = 0;
        while( ros::ok() )
        {

            for ( int leg=0; leg < 6; leg++ ){

                std_msgs::String msg;
                std::stringstream ss;

                ss << "hello leg" << leg << " msg_num: " << count;
                msg.data = ss.str();

                pub_leg[leg].publish( msg ); // Print out
            }

            ros::spinOnce();

            loop_rate.sleep();
            count++;
        }

    }
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

