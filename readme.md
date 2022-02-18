# rslidar to velodyne

convert rslidar point_cloud into velodyne point_cloud(with rings).

## Brief

rslidar's pointcloud don't have property called ring (which ranges from 0 to 16, if a VLP-16 used).So many algorithm depends on it may suffer conflicts using rslidar.

Here I manually convert rslidar point_cloud into velodyne point_cloud.

## Usage 
```
roslaunch rslidar_to_velodyne rslidar_to_velodyne.launch
```

you can change the subscribe topic name and publish topic name in launch file.

For Chinese explanation you can refer: [速腾激光雷达与velodyne激光雷达的数据转换](https://epsavlc.github.io/2019/04/12/rslidar_to_robosense.html)
