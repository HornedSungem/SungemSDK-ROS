# SungemSDK-ROS使用教程
本文主要介绍如何通过ROS使用角蜂鸟的基本示例教程
## 1 配置环境
操作系统 Ubuntu16.04下

### 配置ROS环境 (推荐kinetic版本)
下面给大家介绍配置ROS环境具体的执行过程
  1. 设置当前系统可接受ROS的包,并设置密钥。

  ```bash
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  ```
  2. 检查apt-get更新，安装ROS, rqt, rviz, and robot-generic libraries。

  ```bash
  sudo apt-get update
  sudo apt-get install ros-kinetic-desktop
  ```
  安装不上就科学上网一下
  3. 安装rosdep,rosdep为要编译的源代码安装系统依赖项，并且需要在ROS中运行一些核心组件。

  ```sh
  sudo rosdep init
  rosdep update
  ```

  4. 如果报错多尝试几次，实在不行删除配置文件

  ```bash
  cd /etc/ros/rosdep/sources.list.d
  sudo rm -rf 20-default.list
  ```
  重新执行第3步骤
  5.  设置ROS环境变量到bashrc

  ```bash
  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```
  6. 更新配置

  ```sh
  source /opt/ros/kinetic/setup.bash
  ```

  7. 安装其它依赖包

  ```bash
  sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
  ```

**添加代理方式**：配置完毕代理以后，执行下列命令

  ```
  sudo apt-get -o Acquire::http::proxy="http://127.0.0.1:1080/" update
  ```

参考链接[ROS环境安装](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### 配置SungemSDK环境及资源

  1. 下载SungemSDK基础SDK，保证你的环境安装了git，如果没有安装，请参照百度经验[ubuntu 安装git](https://jingyan.baidu.com/article/915fc414ba51be51394b20c9.html)，git下来角蜂鸟的SDK

  ```
  git clone https://github.com/HornedSungem/SungemSDK.git
  ```
  **如已下载SungemSDK，请更新保证它是最新版本**

  2. 拷贝hs.h接口头文件到/usr/local/include，SungemSDK-ROS需要引用该头文件，所以需拷贝到系统目录下

  ```
  sudo cp -i <workspace>/SungemSDK/api/include/hs.h  /usr/local/include
  ```
  3. 根据您当前系统的架构选择对应文件夹下的libhs.so,拷贝该文件到/usr/local/lib

  ```
  sudo cp -i <workspace>/SungemSDK/api/libs/linux/x86_64/libhs.so /usr/local/lib
  ```

  4. 准备需要的资源文件，model及data文件，git最新SungemSDK-GraphModels,或从百度云盘中下载，并把该目录link到系统下,如果不想放在系统下，你可以修改horned_sungem_launch目录下config下每次模型类型的graph_file_path和category_file_path参数，确保路径下能找到该文件即可。

    ```
    git clone https://github.com/HornedSungem/SungemSDK-GraphModels.git
    sudo ln -s <workspace>/SungemSDK-GraphModels/ /opt/SungemSDK-GraphModels
    ```

### 创建ROS工作空间

  1. 创建一个工作空间，名字自定义

  ```
  mkdir -p ~/catkin_<name>/src
  ```
  2. cd到你当前的工作空间下

  ```
  cd ~/catkin_<name>/src
  ```
  3. 下载SungemSDK-ROS

  ```
  git clone https://github.com/HornedSungem/SungemSDK-ROS.git
  ```
  4. 使用catkin编译生成build和devel文件

  ```
  catkin_make
  ```
  如果编译过程中报错，可根据错误内容对源码进行修改，并查找目录下CmakeList.txt和package.xml是否有误

  5. 执行source命令，依次执行文件中的语句，使刚修改的初始化文件生效

  ```
  source devel/setup.bash
  ```
  6.
  ```
  echo $ROS_PACKAGE_PATH
/home/<user_name>/catkin_<name>/src:/opt/ros/kinetic/share
  ```

## 2 如何使用Sungem-ROS
  配置完上述环境，下面就跟大家介绍下如何使用Sungem-ROS。 根据需求，我们提供了俩种模式：
  1. 使用usb摄像头：
    - 涉及参数：
      - camera:=usb
      - cnn_type :=googlenet (默认为googlenet) :声明使用的模型类型
    - 如果没有配置ROS摄像头，需要先安装环境

      ```
      sudo apt-get install ros-kinetic-usb-cam
      ```
    - 示例：使用mobilenetssd进行检测

      * 打开终端，加载nodelet

        ```
        roslaunch horned_sungem_launch hs_camera.launch cnn_type:=mobilenetssd camera:=usb
        ```
        如果出现错误，需cd到你的catkin工作空间的devel重新source下setup.bash。
        打开新的终端，加载图像
        ```
        roslaunch horned_sungem_launch hs_detection_example.launch camera_topic:="/usb_cam/image_raw"
        ```
        **注意：**因为是使用的usb摄像头，不管模型的类型，**camera_topic是固定的**
  2. 使用角蜂鸟自带摄像头：
    - 涉及参数：
      - camera:=hs (默认为hs) ：如果不声明则使用的是角蜂鸟摄像头
      - pixels:=1080 (默认为360) ：角蜂鸟提供了俩个摄像头分辨率，1920\*1080和640\*360，你可以根据项目，选择适用于自己的分辨率。
    - 示例：下面介绍使用googlenet模型进行分类
      * 打开终端，加载nodelet,接受角蜂鸟360P的图像

        ```
        roslaunch horned_sungem_launch hs_camera.launch cnn_type:=googlenet camera:=hs pixels:=360
        ```

        如果出现错误，需cd到你的catkin工作空间的devel重新source下setup.bash

        打开新的终端，加载图像

        ```
        roslaunch horned_sungem_launch hs_classification_example.launch camera_topic:="/hs/camera/image_raw"
        ```

## 3 参数说明
  在horned_sungem_launch文件夹下，你可以查看所有的launch文件，用于启动多个节点完成复杂需求，您可以根据您的项目需求，对其进行修改丰富，现在给大家介绍下launch文件里现有参数及其用法
  * 常用参数：
    - cnn_type:卷积神经网络类型，默认为googlenet 主要分两类：tensorflow,caffe
      - MobileNetSSD
      - Yolo
      - AlexNet
      - GoogLeNet
      - SqueezeNet
      - Inception_v1,Inception_v2,Inception_v3,Inception_v4
      - MobileNet
    - camera:默认值为hs
      - hs
      - usb
    - pixels:默认为360
      - 360
      - 1080
      - 其他数值现在只评定为1080P
  * 其他参数：
   - device_index:默认为0
   - log_level:默认为0
   - video_device：默认为/dev/video0  使用usb摄像头，如果连接设备过多，需要找到对应的名字进行指定

**注意：** 参数声明的时候都为小写
