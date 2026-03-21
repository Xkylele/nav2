其中/map -> /odom 的tf关系是由 各种 SLAM算法发布的，例如icp、amcl
/odom -> /base_link 的tf关系是由各种lio算法发布的，例如fast_lio,point_lio
对于mid360的点云数据来说，还需要进行
/base_link -> livox_frame 的tf变换，是由 pointcloud_to_laserscan 算法发布的

navigation2的lauch文件中 去掉'use_map_topic': 'true'
不要使用topic话题来发布/map
使用静态地图 launch参数'map'： xxx.yaml

patchelf   --set-rpath "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib"   --force-rpath   /home/lele/Desktop/nav2/install/fast_lio/lib/fast_lio/fastlio_mapping

patchelf   --set-rpath "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib"   --force-rpath   /home/lele/Desktop/nav2/install/icp_registration/lib/icp_registration/icp_registration_node

patchelf   --set-rpath "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib"   --force-rpath   /home/lele/Desktop/nav2/install/terrain_analysis_ext/lib/terrain_analysis_ext/pathNorm

problem：local cost map 一直有问题 point_to_scan没办法直接用 /livox/lidar
最后将point_to_scan中的remapping 改为订阅 fastlio2 发布的/cloud_registered_body
            remappings=[
                ('cloud_in', '/cloud_registered_body'),   # 直接订阅真实点云
                ('scan', '/scan')  # 输出 /scanner/scan
            ],

26.2.24
去掉了nav.sh的icp
每次2Destimate后都无法回到真实位置
最终结果amcl不能收敛 可能是因为/scan是3d 而amcl需要用2d导致没法收敛 
version 1.0 ---- 看能否使用icp来对齐

---version 2.0
遗留bug
problem：local cost map 一直有问题

3.19 修改pointcloud2scan后 局部地图正常


需要修改的配置
nav2param.yaml
registration/icp/config
autopatrol/config

改完mapyaml 要build