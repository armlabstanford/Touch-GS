#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/TimeReference.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include "std_msgs/String.h"