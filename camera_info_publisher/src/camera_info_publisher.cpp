#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

sensor_msgs::CameraInfo createCameraInfo(
    int width, int height, double fx, double fy, double cx, double cy, const std::string& frame_id) {
    sensor_msgs::CameraInfo camera_info;
    camera_info.width = width;
    camera_info.height = height;

    camera_info.K = {fx, 0.0, cx,
                     0.0, fy, cy,
                     0.0, 0.0, 1.0};

    camera_info.D = {0.0, 0.0, 0.0, 0.0, 0.0};

    camera_info.P = {fx, 0.0, cx, 0.0,
                     0.0, fy, cy, 0.0,
                     0.0, 0.0, 1.0, 0.0};

    camera_info.header.frame_id = frame_id;  // Set the frame_id

    return camera_info;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_info_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub_camera_info_0 = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera0/camera_info", 1);
    ros::Publisher pub_camera_info_1 = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera1/camera_info", 1);
    ros::Publisher pub_camera_info_2 = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera2/camera_info", 1);
    ros::Publisher pub_camera_info_3 = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera3/camera_info", 1);
    ros::Publisher pub_camera_info_4 = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera4/camera_info", 1);
    ros::Publisher pub_camera_info_5 = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera5/camera_info", 1);

    ros::Rate loop_rate(10);

    sensor_msgs::CameraInfo camera_info_0 = createCameraInfo(1600, 900, 1266.4172, 1266.4172, 816.2670, 491.5070, "camera0");  // CAM_FRONT
    sensor_msgs::CameraInfo camera_info_1 = createCameraInfo(1600, 900, 1272.8625, 1272.8625, 826.2411, 479.9155, "camera1");  // CAM_FRONT_LEFT
    sensor_msgs::CameraInfo camera_info_2 = createCameraInfo(1600, 900, 1260.8474, 1260.8474, 807.7888, 495.9542, "camera2");  // CAM_FRONT_RIGHT
    sensor_msgs::CameraInfo camera_info_3 = createCameraInfo(1600, 900, 1256.9629, 1256.9629, 792.3768, 492.5482, "camera3");  // CAM_BACK_LEFT
    sensor_msgs::CameraInfo camera_info_4 = createCameraInfo(1600, 900, 1259.9629, 1259.9629, 807.3768, 501.5482, "camera4");  // CAM_BACK_RIGHT
    sensor_msgs::CameraInfo camera_info_5 = createCameraInfo(1600, 900, 809.8911, 809.8911, 829.7774, 481.8849, "camera5");    // CAM_BACK

    while (ros::ok()) {
        ros::Time now = ros::Time::now();

        camera_info_0.header.stamp = now;
        camera_info_1.header.stamp = now;
        camera_info_2.header.stamp = now;
        camera_info_3.header.stamp = now;
        camera_info_4.header.stamp = now;
        camera_info_5.header.stamp = now;

        pub_camera_info_0.publish(camera_info_0);
        pub_camera_info_1.publish(camera_info_1);
        pub_camera_info_2.publish(camera_info_2);
        pub_camera_info_3.publish(camera_info_3);
        pub_camera_info_4.publish(camera_info_4);
        pub_camera_info_5.publish(camera_info_5);

        loop_rate.sleep();
    }

    return 0;
}