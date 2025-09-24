#include "yolox.hpp"
#include <gflags/gflags.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "yolo_detection_node");
    ros::NodeHandle nh("~");

    YOLOXNode yolo_node(nh, argc, argv);
    yolo_node.createRosSubPub();
    ros::spin();

    if (yolo_node.getProfFlag()) {
        yolo_node.printProfiling();
    }

    return 0;
}