# vrx_nav_test  
  
Mapping, navigation simulation for wamv in ROS environment  
  
Based on simulator: bitbucket.org/osrf/vrx  

Package is in developping.
  
## Advance preparation  
### Vrx simulator  
Get source code and compile it follow with the tutorial: [vrx simulator](bitbucket.org/osrf/vrx/wiki/tutorials) 
  
or  
  
sudo apt-get install ros-melodic-vrx*  
  
### Cartographer  
Follow with the guidance of cartographer_ros: [cartographer](google-cartographer-ros.readthedocs.io/en/latest/compilation.html)
  
or  

sudo apt-get install ros-melodic-cartographer*  

### Robot localization  
Robot localization is used to generate two transformation: base_link to odom and odom to map  
    
One launch file uses the branch package of mine, it can be get through:[robot_localization master-devel](github.com/wangzhao9562/robot_localization)   
 
Prototype can be get through: [robot_localization](github.com/cra-ros-pkg/robot_localization)  

or    
  
sudo apt-get install ros-melodic-ros-localization

## 2D mapping
roslaunch vrx_nav_test vrx_carto_2d_online.launch  
roslaunch vrx_nav_test vrx_2d_localization.launch  
or  
roslaunch vrx_nav_test vrx_extend_localization.launch 
