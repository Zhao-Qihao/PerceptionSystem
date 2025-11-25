# UpdateInfo
This is a file for recording update information.

## 2021.08.23
* 提交编号: 868162702fb8123e8f0ee0e8bbcacafab488b0bb  
* 更新等级: master(v1.0.0)
* 更新内容: 初始版本

## 2021.09.07
* 提交编号: 3a4e60e2d3708acd7211f8496228f7d5b811c40b
* 更新等级: develop
* 更新内容: 将LidarFrame.msg分割  
  参考 https://e.gitee.com/xjtu_vcciv/dashboard?issue=I48XZ0  
  LidarFrame消息被分割为三个消息。
  1. 障碍物： 使用新的 LidarObstacles.msg
  2. 道路边界： 使用用 DrivableZone.msg
  3. 栅格图： 使用官方的 sensor_msgs/Image 结构

## 2021.09.08
* 提交编号: 5fa2a4a3c4a0b1113b6e0345fc77706be3806fc7
* 更新等级: develop
* 更新内容: 化简障碍物融合结构体 & 整合IFV结构体  
  参考 https://e.gitee.com/xjtu_vcciv/repos/xjtu_vcciv/kf-msgs/issues/list?issue=I495EZ  
  FusionFrame.msg被重命名为FusionObstacles.msg，并将栅格图和可形式区域部分去除  
  IFVFrame.msg被拆分为IFVObstacles.msg和TrafficLights.msg  
  TrafficLight.msg重新定义结构
 