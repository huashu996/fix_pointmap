# 项目介绍
    本项目是针对代码工具链生成的DDS服务文件所建立的。
 
    能够将多个DDS服务文件进行合并，构建DDS基本服务端以及客户端程序框架。包含生成server.cpp、client.cpp、CMakeLists.txt
 
# 环境依赖
 
 
# 目录结构描述
    ├── ReadMe.md           // 帮助文档
    
    ├── AutoCreateDDS.py    // 合成DDS的 python脚本文件
    
    ├── DDScore             // DDS核心文件库，包含各版本的include、src、lib文件夹，方便合并
    
    │   ├── include_src     // 包含各版本的include、src文件夹
    
    │       ├── V1.0
    
    │           ├── include
    
    │           └── src
    
    │       └── V......
    
    │   └── lib             // 包含各版本的lib文件夹
    
    │       ├── arm64       // 支持arm64系统版本的lib文件夹
    
    │           ├── V1.0
    
    │           └── V......
    
    │       └── x86         // 支持x86系统版本的lib文件夹
    
    │           ├── V1.0
    
    │           └── V......
    
    ├── target              // 合成结果存放的文件夹
    
    └── temp                // 存放待合并的服务的服务文件夹
 
# 使用说明
 
 
 
# 版本内容更新
###### v1.0.0: 
    1.实现gen文件的拷贝、合并
    
    2.实现common文件的合并
    
    3.实现指定版本的include、src、lib文件的拷贝
————————————————
版权声明：本文为CSDN博主「星羽空间」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/qq_25662827/article/details/124440992
# fix_pointmap
定点建图：将激光雷达放在三脚架上，缓慢上下转动生成点云地图 
##1、编译运行 
cd fix_point_slam catkin_make source devel/setup.bash roslaunch pcl_reg pcl_reg.launch 
##2、运行ros包 
rosbag play 1-1.bag rviz 
##3、查看所建立的地图 
pcl_viewer xxxxxx.pcd
