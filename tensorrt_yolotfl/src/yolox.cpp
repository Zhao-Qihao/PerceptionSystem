#include "yolox.hpp"
#include <gflags/gflags.h>
#include <chrono>
#include "yolo_config_parser.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <autoware_msgs/TrafficLightResult.h>
#include <autoware_msgs/TrafficLightResultArray.h>

YOLOXNode::YOLOXNode(ros::NodeHandle& nh, int argc, char* argv[]) 
    : nh_(nh), it_(nh_), init_(true) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    // 获取参数
    nh_.getParam("input_topic", input_topic);
    std::cout<<"input_topic: "<<input_topic<<std::endl;
    nh_.getParam("onnx_model_path", model_path);
    std::cout<<"model_path: "<<model_path<<std::endl;
    nh_.getParam("precision", precision);
    std::cout<<"precision: "<<precision<<std::endl;
    nh_.getParam("nms_thresh", nms_thresh);
    std::cout<<"nms_thresh: "<<nms_thresh<<std::endl;
    nh_.getParam("score_thresh", thresh);
    std::cout<<"thresh: "<<thresh<<std::endl;
    nh_.getParam("num_class", num_class);
    std::cout<<"num_class: "<<num_class<<std::endl;
    nh_.getParam("rgbfile_path", rgbfile_path);
    std::cout<<"rgbfile_path: "<<rgbfile_path<<std::endl;
    nh_.getParam("classname_path", classname_path);
    std::cout<<"classname_path: "<<classname_path<<std::endl;

    class_name_ = get_names(classname_path);

    std::string calibration_images = get_calibration_images();
    const int batch = get_batch_size();
    const int dla = get_dla_id();
    const bool cuda = get_cuda_flg();
    const double scale = get_scale();
    const bool first = get_fisrt_flg();
    const bool last = get_last_flg();
    std::string calibType = get_calib_type();
    bool prof = get_prof_flg();
    double clip = get_clip_value();
    const tensorrt_common::BatchConfig & batch_config = {1, batch/2, batch};
    const size_t workspace_size = (1 << 30);
    tensorrt_common::BuildConfig build_config(
        calibType, dla, first, last,
        prof, clip);
    trt_yolox_ = std::make_unique<tensorrt_yolox::TrtYoloX>(model_path, precision,
     num_class, thresh, nms_thresh, build_config, cuda, calibration_images, scale, "", batch_config, workspace_size);
}

void YOLOXNode::createRosSubPub() {
    pub_image_ = it_.advertise("/detection/image_detector/detected_image", 1);
    pub_tlr_ = nh_.advertise<autoware_msgs::TrafficLightResultArray>("/detection/traffic_light/results", 1);
    // pub_bboxes_3d_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/image_detector/3d_objects_new", 1);

    sub_image_ = it_.subscribe(input_topic, 1, &YOLOXNode::imageCallback, this);
}

void YOLOXNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        auto start_time = std::chrono::system_clock::now();
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;

        if (init_) {
            trt_yolox_->initPreprocessBuffer(image.cols, image.rows);
            init_ = false;
        }

        tensorrt_yolox::ObjectArrays objects;
        trt_yolox_->doInference({image}, objects);
        pubDetectedImage(objects, image, msg->header);
        pubTrafficLightResults(objects, msg->header);
        // pubDetected3DBoxes(objects, msg->header);
        auto end_time = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        ROS_INFO_STREAM("Image Inference time: " << duration.count() << "ms");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void YOLOXNode::pubDetectedImage(const tensorrt_yolox::ObjectArrays& objects, const cv::Mat& image, const std_msgs::Header& header) {
    std::vector<std::vector<int>> colormap = get_colormap(rgbfile_path);
    for (const auto & object : objects[0]) {
        const auto left = object.x_offset;
        const auto top = object.y_offset;
        const auto right = std::clamp(left + object.width, 0, image.cols);
        const auto bottom = std::clamp(top + object.height, 0, image.rows);
        if (colormap.size()) {  // 不同框不同颜色
            char buff[128];
            std::vector<int> rgb = colormap[object.type];
            cv::rectangle(
                image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(rgb[2], rgb[1], rgb[0]), 2);
            sprintf(buff, "%2.0f%%", object.score * 100);
            cv::putText(image, buff, cv::Point(left, top), 0, 0.5, cv::Scalar(rgb[2], rgb[1], rgb[0]), 2);
        } else {
            cv::rectangle(
                image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 3, 8, 2);
        }
    }

    // 将处理后的图像转换回ROS图像消息
    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    out_msg->header = header;
    // 发布带有目标框的图像
    pub_image_.publish(out_msg);
}

// void YOLOXNode::image2point_IPM(tensorrt_yolox::Object& object) {
//     cv::Point2d point = cv::Point2d(object.x_offset + object.width/2, object.y_offset + object.height);
//     std::cout << "point: " << point << std::endl;
//     double Zw = -1.73;

//     // lidar2camera的外参矩阵
//     // cv::Mat R = (cv::Mat_<double>(3, 3) <<
//     //             0.007533745, -0.9999714, -0.000616602,
//     //             0.01480249, 0.0007280733, -0.9998902,
//     //             -0.9998621, 0.007523790, 0.01480755);
//     // cv::Mat t = (cv::Mat_<double>(3, 1) <<
//     //             -0.004069766,
//     //             -0.07631618,
//     //             -0.2717806);
//     cv::Mat K = (cv::Mat_<double>(3, 3) <<
//                 959.791, 0.0, 696.0217, 
//                 0.0, 956.9251, 224.1806, 
//                 0.0, 0.0, 1.0);
//     cv::Mat rightMatrix = R.inv() * K.inv() * (cv::Mat_<double>(3, 1) << point.x, point.y, 1.0);
//     cv::Mat leftMatrix = R.inv() * t;
//     double Zc = (Zw + leftMatrix.at<double>(2)) / rightMatrix.at<double>(2);
//     cv::Mat worldPoint = R.inv() * (Zc * K.inv() * (cv::Mat_<double>(3, 1) << point.x, point.y, 1.0) - t);
//     object.x_3d = worldPoint.at<double>(0);
//     object.y_3d = worldPoint.at<double>(1);
//     object.z_3d = worldPoint.at<double>(2);
//     std::cout << "3D Point in Camera Coordinates: (" << worldPoint.at<double>(0) << ", " << worldPoint.at<double>(1) << ", " << worldPoint.at<double>(2) << ")" << std::endl;
// }

void YOLOXNode::image2point_IPM(tensorrt_yolox::Object& object) {
    // 使用kitti数据集的投影矩阵
    double Z = 1.73;
    cv::Mat pixels = (cv::Mat_<double>(4, 1) << object.x_offset + object.width/2, object.y_offset + object.height, 1.0, 1.0);
    // cv::Mat P = (cv::Mat_<double>(4, 4) <<
    //             959.791,  0.0,      696.0217,  0.0,
    //             0.0,      956.9251, 224.1806,  0.0,
    //             0.0,      0.0,      1.0,       0.0,
    //             0.0,      0.0,      0.0,       1.0);
    cv::Mat P = (cv::Mat_<double>(4, 4) << 
        707.0493, 0.0,      604.0814, 45.75831,
        0.0,      707.0493, 180.5066, -0.3454157,
        0.0,      0.0,      1.0,      0.004981016,
        0.0,      0.0,      0.0,      1.0);   
    // [721.5377, 0.0, 609.5593, 44.85728, 0.0, 721.5377, 172.854, 0.2163791, 0.0, 0.0, 1.0, 0.002745884]
    // cv::Mat P = (cv::Mat_<double>(4, 4) << 
    //     721.5377, 0.0,      609.5593, 44.85728,
    //     0.0,      721.5377, 172.854,  0.2163791,
    //     0.0,      0.0,      1.0,      0.002745884,
    //     0.0,      0.0,      0.0,      1.0);
    cv::Mat inv_proj_matrix = P.inv();
    cv::Mat p_camera = inv_proj_matrix * pixels;
    double K = Z / p_camera.at<double>(1);
    cv::Mat worldPoint = p_camera * K;


    object.x_3d = worldPoint.at<double>(2) + 0.27;
    object.y_3d = -worldPoint.at<double>(0);
    object.z_3d = -Z;
    // 打印3D点坐标
    // std::cout << "3D Point in Camera Coordinates: (" << object.x_3d << ", " << object.y_3d << ", " << Z << ")" << std::endl;
}

// void YOLOXNode::pubDetected3DBoxes(tensorrt_yolox::ObjectArrays& detected_objects, const std_msgs::Header& header) {
//     autoware_msgs::DetectedObjectArray objects;
//     objects.header = header;
//     objects.header.frame_id = "rslidar";
//     for (auto & detected_object : detected_objects[0]) {
//         image2point_IPM(detected_object);

//         autoware_msgs::DetectedObject object;
//         object.header = header;
//         object.header.frame_id = "rslidar";
//         object.label = class_name_[detected_object.type];
//         object.valid = true;
//         object.pose.position.x = detected_object.x_3d;
//         object.pose.position.y = detected_object.y_3d;
//         object.pose.position.z = detected_object.z_3d;
//         geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(0);
//         object.pose.orientation = q;
//         // TODO:根据object的label，修改dimension信息
//         giveDimentions(object);
//         object.score = detected_object.score;
//         if (filter_dist_objects(object)) {
//             objects.objects.push_back(object);     // only label 'car', 'pedestrian', 'cyclist' , 'unknown' are valid
//         }
//     }
//     pub_bboxes_3d_.publish(objects);
// }

bool YOLOXNode::filter_dist_objects(autoware_msgs::DetectedObject& object) {
    double dist = sqrt(pow(object.pose.position.x, 2) + pow(object.pose.position.y, 2));
    return (dist < 50);
}

void YOLOXNode::giveDimentions(autoware_msgs::DetectedObject& object) {
    if (object.label == "car") {
        object.dimensions.y = 1.8;
        object.dimensions.x = 3.9;
        object.dimensions.z = 1.6;
    }
    else if (object.label == "truck") {
        object.dimensions.y = 3.2;
        object.dimensions.x = 3.8;
        object.dimensions.z = 2.5;
    }
    else if (object.label == "bus") {
        object.dimensions.y = 3.2;
        object.dimensions.x = 6.8;
        object.dimensions.z = 2.5;
    }
    else if (object.label == "bicycle" || object.label == "motorcycle") {
        object.dimensions.y = 0.6;
        object.dimensions.x = 1.5;
        object.dimensions.z = 1.5;
    }
    else if (object.label == "pedestrian") {
        object.dimensions.y = 0.6;
        object.dimensions.x = 0.8;
        object.dimensions.z = 1.8;
    }
    else if (object.label == "animal" || object.label == "unknown") {
        object.label = "barrier";
        object.dimensions.y = 0.6;
        object.dimensions.x = 0.8;
        object.dimensions.z = 1.8;
    }
}

void YOLOXNode::pubTrafficLightResults(const tensorrt_yolox::ObjectArrays& detected_objects, const std_msgs::Header& header) {
    autoware_msgs::TrafficLightResultArray out;
    out.header = header;
    // TODO: 1. 根据name修改msg   2. 检测到多个红绿灯怎么办， 可以根据检测框位置或其他逻辑设置？
    int id = 0;
    for (const auto& d : detected_objects[0]) {
        autoware_msgs::TrafficLightResult r;
        const std::string& label = class_name_[d.type];
        std::cout << "label: " << label << std::endl;

        int state = 2;
        std::string state_str = "UNKNOWN";
        if (label == "green" || label == "Green") {
            state = 1; state_str = "GREEN";
        } else if (label == "red" || label == "Red") {
            state = 0; state_str = "RED";
        } else if (label == "yellow" || label == "Yellow" || label == "amber") {
            state = 0; state_str = "YELLOW";
        }

        r.light_id = id++;     // 无法确定真实ID时用递增占位
        r.recognition_result = state;
        r.recognition_result_str = state_str;
        r.lane_id = -1;        // 暂无车道ID，设为-1
        out.results.push_back(r);
    }

    pub_tlr_.publish(out);
}

bool YOLOXNode::getProfFlag() const {
    return prof_;
}

void YOLOXNode::printProfiling() const {
    trt_yolox_->printProfiling();
}