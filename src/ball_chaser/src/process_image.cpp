#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    //Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    //Call the DriveToTarget service
    client.call(srv);
    if(!client.call(srv))
	ROS_ERROR("Failed to call service DriveToTarget");
}


// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera


    //Count number of white pixels
    int white_count = 0;
    //int total_count = 0;
    
    float total_pos = 0;
    //uint32 height         # number of rows
    //uint32 width          # number of columns
    //uint32 step           # Full row length in bytes
    //uint8[] data          # actual matrix data, size is (step * rows)
    
    //img.data consists of 3 values per colour unit, red, green, blue
    //Loop through every 3rd value. Each 3 values is a pixel
    //For each pixel
    for (int i = 0; i + 2 < img.data.size(); i+=3)
    {

        //Check that all 3 values in a pixel are 255
        // To get white, R = 255, G = 255, B = 255
        //values are located sequentially 
        if (img.data[i] == white && img.data[i + 1] == white && img.data[i + 2] == white)
	    {

		    //Check if white pixels are present. add 1 to white pixel count rgb = 255
            white_count++;

            //Find position of pixel
            //img.step = 3 * img.width
            //
            total_pos += (i % img.step) / 3 ;
	    }
    }

    //Find average position of all white pixels to find centre of white ball
    float avg_pos = total_pos / white_count; 

    if (white_count == 0)
    {
        drive_robot(0.0, 0.0);
    }

    //if white pixels are present, check position of white pixels
    //NOTE: Image into fifths instead of thirds

    else 
    {
        if (avg_pos < img.width * 2 / 5) 
        {
            drive_robot(0.2, 0.2);
        }
        else if (avg_pos > img.width * 3 / 5)
        {
            drive_robot(0.2, -0.2);
        }
        else
        {
            drive_robot(0.2, 0);
        }
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
