
# Lidar to Camera Extrinsic Calibration
First pull the docker image:
```bash
docker pull koide3/direct_visual_lidar_calibration:humble
```
Then activate the X server connection port:
```bash 
xhost +
```
and run the container with GUI support:
```bash
docker run \
  -it \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v /path/to/input/bags:/tmp/input_bags \
  -v /path/to/save/result:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:humble 
```

Inside the container generate images from the LiDAR data (Preprocessing):
```bash
ros2 run direct_visual_lidar_calibration preprocess /path/to/bag/dir /path/to/result/dir \
  --image_topic /camera/color/image_raw \
  --points_topic /utlidar/cloud \
  --camera_model plumb_bob \
  --camera_intrinsics 379.7099304199219,320.3064270019531,379.3695983886719,243.11753845214844 \
  --camera_distortion_coeffs -0.057967256754636765,0.0704321563243866,-0.00015285948757082224,0.0006057045538909733,-0.022366832941770554
```
then manually select the correspondences between the 3D pointcloud and the image:

```bash
ros2 run direct_visual_lidar_calibration initial_guess_manual /path/to/result/dir
```
Finally perform a refinement step by running:

```bash
ros2 run direct_visual_lidar_calibration calibrate /path/to/result/dir
```
[![Alt text](https://img.youtube.com/vi/YOUTUBE_VIDEO_ID/0.jpg)](https://www.youtube.com/watch?v=FTlC9RwEVxY&t=43s)

# Source
This procedure has been taken from [direct_visual_lidar_calibration toolbox](https://koide3.github.io/direct_visual_lidar_calibration/).

