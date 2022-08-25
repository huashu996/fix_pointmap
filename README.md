# fix_pointmap
定点建图：将激光雷达放在三脚架上，缓慢上下转动生成点云地图 
##1、编译运行 
cd fix_point_slam catkin_make source devel/setup.bash roslaunch pcl_reg pcl_reg.launch 
##2、运行ros包 
rosbag play 1-1.bag rviz 
##3、查看所建立的地图 
pcl_viewer xxxxxx.pcd
