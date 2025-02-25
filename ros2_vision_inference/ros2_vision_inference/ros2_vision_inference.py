#!/usr/bin/env python3

import onnxruntime as ort
import numpy as np
import autoware_msgs.msg
import rospy
import cv2
from utils.ros_util import ROSInterface
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, CompressedImage
from visualization_msgs.msg import MarkerArray
import threading
from numba import jit
from seg_labels import PALETTE
import time
import tf2_ros
import geometry_msgs.msg
import autoware_msgs
import warnings
warnings.filterwarnings("ignore")

MONO3D_NAMES = ['car', 'truck', 'bus', 
                'trailer', 'construction_vehicle',
                'pedestrian', 'motorcycle', 'bicycle',
                'traffic_cone', 'barrier']

COLOR_MAPPINGS = {
    'car' : (  0,  0,142),  'truck': (  0,  0, 70) ,
    'bus': (  0, 60,100), 'trailer': (  0,  0,110),
    'construction_vehicle':  (  0,  0, 70), 'pedestrian': (220, 20, 60),
    'motorcycle': (  0,  0,230), 'bicycle': (119, 11, 32),
    'traffic_cone': (180,165,180), 'barrier': (190,153,153)
}

@jit(nopython=True, cache=True)
def ColorizeSeg(pred_seg, rgb_image, opacity=1.0, palette=PALETTE):
    color_seg = np.zeros((pred_seg.shape[0], pred_seg.shape[1], 3), dtype=np.uint8)
    h, w = pred_seg.shape
    for i in range(h):
        for j in range(w):
            color_seg[i, j] = palette[pred_seg[i, j]]
    new_image = rgb_image * (1 - opacity) + color_seg * opacity
    new_image = new_image.astype(np.uint8)
    return new_image

def normalize_image(image, rgb_mean = np.array([0.485, 0.456, 0.406]), rgb_std  = np.array([0.229, 0.224, 0.225])):
    image = image.astype(np.float32)
    image = image / 255.0
    image = image - rgb_mean
    image = image / rgb_std
    return image


class BaseInferenceThread(threading.Thread):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.build_model(*args, **kwargs)
    
    def build_model(self, onnx_path, gpu_index=0):
        providers = [("CUDAExecutionProvider", {"cudnn_conv_use_max_workspace": '0', 'device_id': str(gpu_index)})]
        sess_options = ort.SessionOptions()
        self.ort_session = ort.InferenceSession(onnx_path, providers=providers, sess_options=sess_options)
        input_shape = self.ort_session.get_inputs()[0].shape # [1, 3, h, w]
        self.inference_h = input_shape[2]
        self.inference_w = input_shape[3]
    
    def set_inputs(self, image, P=None):
        self.image = image
        self.P = P

    def resize(self, image, P=None):
        self.h0, self.w0 = image.shape[0:2]
        scale = min(self.inference_h / self.h0, self.inference_w / self.w0)
        self.scale = scale
        self.h_eff = int(self.h0 * scale)
        self.w_eff = int(self.w0 * scale)
        final_image = np.zeros([self.inference_h, self.inference_w, 3])
        final_image[0:self.h_eff, 0:self.w_eff] = cv2.resize(image, (self.w_eff, self.h_eff),
                                                    interpolation=cv2.INTER_LINEAR)
        if P is None:
            return final_image
        else:
            P = P.copy()
            P[0:2, :] = P[0:2, :] * scale
            return final_image, P

    def deresize(self, seg):
        seg = seg[0:self.h_eff, 0:self.w_eff]
        seg = cv2.resize(seg, (self.w0, self.h0), interpolation=cv2.INTER_NEAREST)
        return seg

    def run(self):
        raise NotImplementedError

    def join(self):
        threading.Thread.join(self)
        threading.Thread.__init__(self)
        return self._output

class Mono3D(BaseInferenceThread):
    def run(self):
        global MONO3D_NAMES
        object = autoware_msgs.msg.DetectedObject()

        start_time = time.time()
        resized_image, resized_P = self.resize(self.image, self.P)
        input_numpy = np.ascontiguousarray(np.transpose(normalize_image(resized_image), (2, 0, 1))[None], dtype=np.float32)
        P_numpy = np.array(resized_P, dtype=np.float32)[None]
        outputs = self.ort_session.run(None, {'image': input_numpy, 'P2': P_numpy})
        # print('shape:', outputs.shape)
        print('value:', outputs)
        scores = np.array(outputs[0]) # N
        bboxes = np.array(outputs[1]) # N, 12
        cls_indexes = outputs[2] # N

        cls_names = [MONO3D_NAMES[cls_index] for cls_index in cls_indexes]

        objects = []
        N = len(bboxes)
        
        
        for i in range(N):
            obj = {}
            obj['whl'] = bboxes[i, 7:10]
            obj['theta'] = bboxes[i, 11]
            obj['score'] = scores[i]
            obj['type_name'] = cls_names[i]
            obj['xyz'] = bboxes[i, 4:7]
            objects.append(obj)
        self._output = objects
        print(f"mono3d runtime: {time.time() - start_time}")

class SegmentationThread(BaseInferenceThread):
    def run(self):
        start_time = time.time()
        resized_image = self.resize(self.image)
        input_numpy = np.ascontiguousarray(np.transpose(normalize_image(resized_image), (2, 0, 1))[None], dtype=np.float32)
        outputs = self.ort_session.run(None, {'input': input_numpy})
        self._output = self.deresize(np.array(outputs[0][0], np.uint8))
        print(f"segmentation runtime: {time.time() - start_time}")

class MonodepthThread(BaseInferenceThread):
    def run(self):
        start_time = time.time()
        resized_image, resized_P = self.resize(self.image, self.P)
        input_numpy = np.ascontiguousarray(np.transpose(normalize_image(resized_image), (2, 0, 1))[None], dtype=np.float32)
        P_numpy = np.array(resized_P, dtype=np.float32)[None]
        outputs = self.ort_session.run(None, {'image': input_numpy, 'P2': P_numpy})
        self._output = self.deresize(outputs[0][0, 0])
        print(f"monodepth runtime: {time.time() - start_time}")

class VisionInferenceNode():
    def __init__(self):
        self.ros_interface = ROSInterface("VisionInferenceNode")
        self._read_params()
        self._init_model()
        self._init_static_memory()
        self._init_topics()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Initialization Done")
        self.ros_interface.spin()

    def _read_params(self):
        rospy.loginfo("Reading parameters...")

        self.mono3d_flag = self.ros_interface.read_one_parameters("MONO3D_FLAG", True)
        self.seg_flag = self.ros_interface.read_one_parameters("SEG_FLAG", True)
        self.monodepth_flag = self.ros_interface.read_one_parameters("MONODEPTH_FLAG", True)
        
        
        if self.mono3d_flag:
            self.mono3d_weight_path  = self.ros_interface.read_one_parameters("MONO3D_CKPT_FILE",
                                    "/home/yliuhb/vision_factory/weights/mono3d.onnx")
            self.mono3d_gpu_index    = int(self.ros_interface.read_one_parameters("MONO3D_GPU_INDEX", 0))

        if self.seg_flag:
            self.seg_weight_path = self.ros_interface.read_one_parameters("SEG_CKPT_FILE",
                                        "/home/yliuhb/vision_factory/weights/seg.onnx")
            self.seg_gpu_index   = int(self.ros_interface.read_one_parameters("SEG_GPU_INDEX", 0))
        
        if self.monodepth_flag:
            self.monodepth_weight_path = self.ros_interface.read_one_parameters("MONODEPTH_CKPT_FILE",
                                        "/home/yliuhb/vision_factory/weights/monodepth.onnx")
            self.monodepth_gpu_index   = int(self.ros_interface.read_one_parameters("MONODEPTH_GPU_INDEX", 0))
        
        self.gpu_index = int(self.ros_interface.read_one_parameters("GPU", 0))
        self.seg_opacity = float(self.ros_interface.read_one_parameters("opacity", 0.9))

    def _init_model(self):
        rospy.loginfo("Initializing model...")
        if self.mono3d_flag:
            self.mono3d_thread = Mono3D(self.mono3d_weight_path, gpu_index=self.mono3d_gpu_index)
        if self.seg_flag:
            self.seg_thread    = SegmentationThread(self.seg_weight_path, gpu_index=self.seg_gpu_index)
        if self.monodepth_flag:
            self.monodepth_thread = MonodepthThread(self.monodepth_weight_path, gpu_index=self.monodepth_gpu_index)
        rospy.loginfo("Model Done")
    
    def _init_static_memory(self):
        rospy.loginfo("Initializing static memory...")
        self.frame_id = "frame_cam00"
        self.P = np.zeros([3, 4])
        self.P[0:3, 0:3] = np.array(
            [ 592.9087,  0,  521.0287,
           0,  593.1265,  398.2069,
           0,         0,    1.0000 ]
        ).reshape(3, 3)
        self.num_objects = 0
    

    def _init_topics(self):
        self.bbox_publish = self.ros_interface.create_publisher(MarkerArray, "mono3d/bbox", queue_size=10)
        self.ros_interface.create_publisher(Image, "seg_image", queue_size=10)
        self.ros_interface.create_publisher(Image, "depth_image", queue_size=10)
        self.ros_interface.create_publisher(PointCloud2, "point_cloud", queue_size=10)

        self.ros_interface.create_subscription(CameraInfo, "/camera_info", self.camera_info_callback)
        self.ros_interface.create_subscription(Image, "/image_raw", self.camera_callback, buff_size=2**24, queue_size=1)
        self.ros_interface.create_subscription(CompressedImage, "/image_compress", self.compressed_camera_callback, buff_size=2**24, queue_size=1)
        self.ros_interface.clear_all_bbox()


    def camera_info_callback(self, msg:CameraInfo):
        self.P = np.zeros((3, 4))
        self.P[0:3, 0:3] = np.array(msg.K).reshape((3, 3))
        # print(self.P)
        self.frame_id = msg.header.frame_id

    def processing_image(self, image:np.array):
        starting = time.time()
        if self.mono3d_flag:
            self.mono3d_thread.set_inputs(image, self.P.copy())
            self.mono3d_thread.start()
        if self.seg_flag:
            self.seg_thread.set_inputs(image)
            self.seg_thread.start()
        if self.monodepth_flag:
            self.monodepth_thread.set_inputs(image, self.P.copy())
            self.monodepth_thread.start()


        objects = self.mono3d_thread.join() if self.mono3d_flag else None
        seg = self.seg_thread.join() if self.seg_flag else None
        depth = self.monodepth_thread.join() if self.monodepth_flag else None

        rospy.loginfo(f"Total runtime: {time.time() - starting}")

        # publish objects
        if self.mono3d_flag:
            marker_array = MarkerArray()
            for i, obj in enumerate(objects):
                class_name = obj['type_name']
                # print(i, ", ", obj)
                # print(self.frame_id)
                # Convert 3D bounding box to velo_link coordinate system
                try:
                    T_velo_camera = self.get_transform_matrix('velo_link', self.frame_id)
                    bbox_vertices_camera = self.compute_bbox_vertices(obj)
                    bbox_vertices_velo = self.transform_points(bbox_vertices_camera, T_velo_camera)
                    obj['bbox_vertices_velo'] = bbox_vertices_velo

                    # print(f"Object {i} bbox vertices in velo_link: {bbox_vertices_velo}")
                except Exception as e:
                    rospy.logerr(f"Failed to transform bounding box: {e}")
                # 更新 obj 的 xyz 和 whl
                update_obj_from_bbox_vertices(obj)
                marker = self.ros_interface.object_to_marker(obj,
                                                            # self.frame_id,
                                                            "velo_link",
                                                            i,
                                                            color=COLOR_MAPPINGS[class_name],
                                                            duration=0.2) # color maps
                marker_array.markers.append(marker)
            self.bbox_publish.publish(marker_array)

        # publish colorized seg
        if self.seg_flag:
            seg_image = ColorizeSeg(seg, image, opacity=self.seg_opacity)
            self.ros_interface.publish_image(seg_image[:, :, ::-1], image_topic="seg_image", frame_id=self.frame_id)

        # publish depth
        if self.monodepth_flag:
            self.ros_interface.publish_image(depth, image_topic="depth_image", frame_id=self.frame_id)

        # publish colorized point cloud
        if self.seg_flag and self.monodepth_flag:
            kernel = np.ones((7, 7), np.uint8)
            mask = np.logical_not(seg == 23)
            mask = cv2.erode(mask.astype(np.uint8), kernel, iterations=1).astype(np.bool_)

            point_cloud = self.ros_interface.depth_image_to_point_cloud_array(depth, self.P[0:3, 0:3], rgb_image=seg_image, mask=mask)
            mask = (point_cloud[:, 1] > -3.5) * (point_cloud[:, 1] < 5.5) * (point_cloud[:, 2] < 80)
            point_cloud = point_cloud[mask]
            self.ros_interface.publish_point_cloud(point_cloud, "point_cloud", frame_id=self.frame_id, field_names='xyzrgb')
    
    def compressed_camera_callback(self, msg:CompressedImage):
        if self.P is None:
            rospy.loginfo("Waiting for camera info...", throttle_duration_sec=0.5)
            return # wait for camera info

        cv_bridge = self.ros_interface.cv_bridge
        image = cv_bridge.compressed_imgmsg_to_cv2(msg)[..., ::-1] #[BGR] -> [RGB]
        self.processing_image(image)

    def camera_callback(self, msg:Image):
        if self.P is None:
            rospy.loginfo("Waiting for camera info...", throttle_duration_sec=0.5)
            return # wait for camera info
        height = msg.height
        width  = msg.width
        
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))[:, :, ::-1]
        self.processing_image(image)
    def get_transform_matrix(self, target_frame, source_frame):
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
            translation = trans.transform.translation
            rotation = trans.transform.rotation
            T = self.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
            T[:3, 3] = [translation.x, translation.y, translation.z]
            return T
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return None

    def compute_bbox_vertices(self, obj):
        x, y, z = obj['xyz']
        w, h, l = obj['whl']
        theta = obj['theta']

        # Compute the 8 vertices of the 3D bounding box in the camera coordinate system
        c = np.cos(theta)
        s = np.sin(theta)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        half_w = w / 2
        half_h = h / 2
        half_l = l / 2

        vertices = np.array([
            [-half_w, -half_h, -half_l],
            [ half_w, -half_h, -half_l],
            [ half_w,  half_h, -half_l],
            [-half_w,  half_h, -half_l],
            [-half_w, -half_h,  half_l],
            [ half_w, -half_h,  half_l],
            [ half_w,  half_h,  half_l],
            [-half_w,  half_h,  half_l]
        ])

        vertices = np.dot(vertices, R.T) + np.array([x, y, z])
        return vertices

    def transform_points(self, points, transform_matrix):
        # Add homogeneous coordinates
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        # Transform points
        transformed_points_homogeneous = np.dot(transform_matrix, points_homogeneous.T).T
        # Remove homogeneous coordinates
        transformed_points = transformed_points_homogeneous[:, :3]
        return transformed_points

    def quaternion_matrix(self, quaternion):
        """Return homogeneous rotation matrix from quaternion.

        >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
        >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
        True
        >>> M = quaternion_matrix([1, 0, 0, 0])
        >>> numpy.allclose(M, numpy.identity(4))
        True
        >>> M = quaternion_matrix([0, 1, 0, 0])
        >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
        True
        """
        q = np.array(quaternion, dtype=np.float64, copy=True)
        n = np.dot(q, q)
        if n < np.finfo(float).eps * 4.0:
            return np.identity(4)
        q *= np.sqrt(2.0 / n)
        q = np.outer(q, q)
        return np.array([
            [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
            [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
            [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
            [                0.0,                 0.0,                 0.0, 1.0]])      

def update_obj_from_bbox_vertices(obj):
    # 提取边界框顶点
    bbox_vertices_velo = obj['bbox_vertices_velo']
    
    # 计算中心位置 (xyz)
    center_x = np.mean(bbox_vertices_velo[:, 0])
    center_y = np.mean(bbox_vertices_velo[:, 1])
    center_z = np.mean(bbox_vertices_velo[:, 2])
    obj['xyz'] = [center_x, center_y, center_z]
    
    # 计算尺寸 (whl)
    min_x = np.min(bbox_vertices_velo[:, 0])
    max_x = np.max(bbox_vertices_velo[:, 0])
    min_y = np.min(bbox_vertices_velo[:, 1])
    max_y = np.max(bbox_vertices_velo[:, 1])
    min_z = np.min(bbox_vertices_velo[:, 2])
    max_z = np.max(bbox_vertices_velo[:, 2])
    
    width = max_x - min_x
    height = max_y - min_y
    length = max_z - min_z
    obj['whl'] = [width, height, length]

def main(args=None):
    VisionInferenceNode()
    

if __name__ == "__main__":
    main()
