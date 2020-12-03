# NCTU-SDC-course-2020
NCTU 2020 self-driving cars course offered by prof. Bob Wang 

# Configurations
locate the launch file competition1.launch inside the launch folder. In line 7, you may set the desired path for the itri_map.pcd file
```launch
<param name="file" type="str" value="<desired path set here>/itri_map.pcd" />
```
for the bag file - sdc_localization_1.bag, it is located in the config folder, you can also change its path to a desired location by changing the launch file in line 5:
```launch
<node pkg="rosbag" type="play" name="player" output="screen" args="-r 0.03 --clock <desired path set here>/sdc_localization_1.bag"/>
```

# Launch
to run file, type in command as below
```bash
$ roslaunch midterm_competition1 competition1
```

# Main code
the main code is located in the src folder - competition1.cpp



