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

    ros::Publisher pub_leg1;

    ros::Rate loop_rate;

    //=====================================================
    // Class constructor
    gait_update_node() : loop_rate(10) {

        LOG( "calling constructor" )

        // Publishes leg states
        pub_leg1 = node.advertise<std_msgs::String>("leg_states0", 1);

        // ros::Rate loop_rate(10);   // Controls how fast the node loops

    };

    //=====================================================
    // Function to update gait state
    void update_gait( ){

        int count = 0;
        while( ros::ok() )
        {
            std_msgs::String msg;
            std::stringstream ss;

            ss << "hello world " << count;

            msg.data = ss.str();

            pub_leg1.publish( msg );

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

