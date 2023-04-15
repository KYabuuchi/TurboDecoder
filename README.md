# TurboDecoder

## How to build

I believe the following command will install `libjpeg-turbo`

`rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO`.

<details><summary>how to install libjepeg-turbo manually</summary><div>

`sudo apt install libturbojpeg libturbojpeg0-dev`

</div></details>

## Supported options

|  source |  x1                | scale              | crop               | crop & scale       |
| :-----: | :----------------: | :----------------: | :----------------: | :----------------: |
|  RGB    | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
|  Bayer  | :heavy_check_mark: |                    | :heavy_check_mark: |                    |


## How to run

### two nodes

#### 1/1 decompression with visualization

`ros2 launch turbo_decoder compare.launch.xm show:=true`

#### 1/2 decompression

`ros2 launch turbo_decoder compare.launch.xml scale_denom:=4`

#### 1/4 decompression with visualization

`ros2 launch turbo_decoder compare.launch.xml scale_denom:=4 show:true`

### one node

#### 1/4 decompression with visualization

`ros2 launch turbo_decoder demo.launch.xml scale_denom:=4`

### Bayer

**NOTE:** turbo_decoder only supports 1/1 decompression
(does not support 1/2 nor 1/4 decompression)

#### 1/1 decompression with visualization

`ros2 launch turbo_decoder bayer.launch.xml show:=true`
