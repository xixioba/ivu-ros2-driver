# Innovusion ros2 manual

## **Install ROS2**

The following official versions are verified to support, of course **Rolling distribution** is also supported, we are also compatible with docker ros2

| Distro                                                       | Release date   | Logo                                                         | EOL date      |
| ------------------------------------------------------------ | -------------- | ------------------------------------------------------------ | ------------- |
| [Humble Hawksbill](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html) | May 23rd, 2022 | ![Humble logo](https://docs.ros.org/en/humble/_images/humble-small.png) | May 2027      |
| [Galactic Geochelone](https://docs.ros.org/en/humble/Releases/Release-Galactic-Geochelone.html) | May 23rd, 2021 | ![Galactic logo](https://docs.ros.org/en/humble/_images/galactic-small.png) | November 2022 |
| [Foxy Fitzroy](https://docs.ros.org/en/humble/Releases/Release-Foxy-Fitzroy.html) | June 5th, 2020 | ![Foxy logo](https://docs.ros.org/en/humble/_images/foxy-small.png) | May 2023      |

## Run innovusion node

- **Build** package

  ```bash
  colcon build [--packages-select innovusion]
  ```

- **Activate** enviroment

  ```
  source install/setup.bash
  ```

  - (Optional)Enable ROS2 share memory

    ```bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=`pwd`/shm_profile.xml 
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ```

- **Run**

  > Connect the lidar and confirm the **lidar_ip** in `launch/ivu_pc2.py`, default(172.168.1.10)

  - ros2 **run**
    ```bash
    ros2 run innovusion <publisher|subscriber>
    ```

  -  ros2 **launch**

    > For parameter modification, please refer to `launch/ivu_pc2.py`
    >

    ```bash
    ros2 launch innovusion ivu_pc2.py
    ```
