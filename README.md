#for pointpillar
1.删除预先生成的engine
cd src/pointpillar/model
rm pointpillar.onnx.cache

2.进行推理及其可视化
roslaunch pointpillar point_pillars.launch

3.播放bag包
rosbag  play 105.bag -r 0.5

#for center_point 
1.删除预先生成的engine, 除onnx以外的都删除
cd ./src/center_point/model
rm *.log *.plan *.8503

2.生成当前tensorrt使用enginer
cd ./src/center_point
sh ./tool/build.trt.sh

3.将点云融合生成融合点云
roslaunch obtain_sweeps_llidars obtain_sweeps_llidars.launch

4.进行推理及其可视化
roslaunch center_point center_point.launch

5.播放bag包
rosbag  play 103.bag -r 0.5

#for bevfusion
1.删除预先生成的engine, 生成当前tensorrt使用enginer
cd src/bevfusion/model
rm -r build
cd ..
sh ./tool/environment.sh
sh ./tool/build_trt_engine.sh

如果出现编译不过去，使用官网进行模型生成

2.将点云融合生成融合点云
roslaunch obtain_sweeps_llidars obtain_sweeps_llidars.launch

3.进行推理及其可视化
roslaunch multi_data_fusion multi_data_fusion.launch

4.播放bag包
rosbag  play 103.bag -r 0.5