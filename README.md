# vrx_nav_test  
  
Mapping, navigation simulation for wamv in ROS environment  
  
Based on simulator: bitbucket.org/osrf/vrx  
  
## Advance preparation  
### Vrx simulator  
Get source code and compile it follow with the tutorial: bitbucket.org/osrf/vrx/wiki/tutorials 
  
or  
  
sudo apt-get install ros-melodic-vrx*  
  
### Cartographer  
Follow with the guidance of cartographer_ros: google-cartographer-ros.readthedocs.io/en/latest/compilation.html
  
or  

sudo apt-get install ros-melodic-cartographer*  

### Robot localization  
Robot localization is used to generate two transformation: base_link to odom and odom to map  
    
One launch file uses the branch package of mine, it can be get through: 
 
Prototype can be get through:  

or    
  
sudo apt-get install ros-melodic-ros-localization

 
