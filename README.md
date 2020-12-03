# NCTU-SDC-course-2020
NCTU 2020 self-driving cars course offered by prof.Bob Wang 

# Competition 1 - ITRI Dataset
dataset running with itri map 

# Configurations 
locate the .launch file competition1.launch inside the launch folder. In line 7, you may set the desired path for the itri_map.pcd file:
```launch
<param name="file" type="str" value="<desired path set here>/itri_map.pcd" />
```
for the .bag file - sdc_localization_1.bag, it is located in the config folder, you can also change its path to a desired location by changing the .launch file in line 5:
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

# Competition 2 - NuScenes2 Dataset
dataset running with NuScenes map and bag file 2

# Configurations 
locate the .launch file competition1.launch inside the launch folder. In line 8, you may set the desired path for the itri_map.pcd file:
```launch
<param name="file" type="str" value="/home/alex/catkin_ws/src/midterm_competition2/src/downsampled.pcd" />
```
note that this .pcd file (downsampled.pcd) is a preprocessed version of the original nuscenes map. if you wish to run the code with the original nuscenes_map.pcd file, please locate the competition2.cpp file in the src folder.
comment out line 151-196 and uncomment line 200-254.

for the .bag file - sdc_localization_2.bag, it is located in the config folder, you can also change its path to a desired location by changing the .launch file in line 9:
```launch
<node pkg="rosbag" type="play" name="player" output="screen" args="-r 0.03 --clock $(find midterm_competition2)/config/sdc_localization_2.bag" launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' "/>
```

# Launch 
to run file, type in command as below
```bash
$ roslaunch midterm_competition2 competition2
```

# Main code 
the main code is located in the src folder - competition2.cpp

# Competition 3 - NuScenes2 Dataset
dataset running with NuScenes map and bag file 3

# Configurations 
locate the .launch file competition1.launch inside the launch folder. In line 8, you may set the desired path for the itri_map.pcd file:
```launch
<param name="file" type="str" value="/home/alex/catkin_ws/src/midterm_competition3/src/downsampled.pcd" />
```
note that this .pcd file (downsampled.pcd) is a preprocessed version of the original nuscenes map. if you wish to run the code with the original nuscenes_map.pcd file, please locate the competition2.cpp file in the src folder.
comment out line 151-196 and uncomment line 200-254.

for the .bag file - sdc_localization_2.bag, it is located in the config folder, you can also change its path to a desired location by changing the .launch file in line 9:
```launch
<node pkg="rosbag" type="play" name="player" output="screen" args="-r 0.03 --clock $(find midterm_competition3)/config/sdc_localization_3.bag" launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' "/>
```

# Launch 
to run file, type in command as below
```bash
$ roslaunch midterm_competition2 competition2
```

# Main code 
the main code is located in the src folder - competition3.cpp


