#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <iostream>

#define LED_FILE "/sys/class/gpio/gpio206/value"
#define FREQUENCY 2

using namespace std;

void write(ofstream &file, char character)
{
    file.seekp(0);
    file << character;
}

void getValue(const std_msgs::Int8::ConstPtr &value)
{
    static ros::Rate rate(FREQUENCY);
    static ofstream file(LED_FILE);
    switch ( value->data )
    {
        /* LED OFF */
        case 0:
            write(file, '0');
            rate.sleep();
            rate.sleep();
            break;
        /* LED BLINKING */
        case 1:
            write(file, '0');
            rate.sleep();
            write(file, '1');
            rate.sleep();
            break;
        /* LED ON */
        case 2:
            write(file, '1');
            rate.sleep();
            rate.sleep();
            break;
        default:
            break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "led_handler");
    ros::NodeHandle n;

    ros:: Subscriber sub = n.subscribe("led_value", 1000, getValue);
    ros::spin();
    return 0;
}
