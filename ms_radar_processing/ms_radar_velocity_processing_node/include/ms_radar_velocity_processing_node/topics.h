//
// Created by davidrobbins on 11/28/23.
//

#ifndef TOPICS_H
#define TOPICS_H

#include <string>

namespace ms_radar_velocity_processing_node::topics
{
    const std::string ROS_INPUT_RADAR_TOPIC_NAME = "/radar_pcl";

    const std::string ROS_OUTPUT_RADAR_VELOCITY_TOPIC_NAME = "cv7_ins/ext/velocity_body";
    const std::string ROS_OUTPUT_RADAR_VELOCITY_VIZ_TOPIC_NAME = "/radar_velocity_viz";

    const std::vector TOPICS_TO_RECORD = {ROS_OUTPUT_RADAR_VELOCITY_TOPIC_NAME};
}


#endif //TOPICS_H
