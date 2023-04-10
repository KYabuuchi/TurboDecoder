# TurboDecoder

## How to build

I believe the following command will install `libjpeg-turbo`

`rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO`.

<details><summary>how to install libjepeg-turbo manually</summary><div>

`sudo apt install libturbojpeg libturbojpeg0-dev`

</div></details>

## How to run

#### x1 decompression with visualization

`ros2 launch turbo_decoder compare.launch.xm show:=true`

#### 1/2 decompression

`ros2 launch turbo_decoder compare.launch.xml scale_denom:=4`

#### 1/4 decompression with visualization

`ros2 launch turbo_decoder compare.launch.xml scale_denom:=4 show:true`
