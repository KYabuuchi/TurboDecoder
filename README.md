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

### Bayer

**NOTE:** For bayer pattern, turbo_decoder only supports 1/1 decompression
(does not support 1/2 nor 1/4 decompression)

#### 1/1 decompression with visualization

`ros2 launch turbo_decoder compare.launch.xm show:=true bayer:=true`

## Evaluation

average time of 50 topics;
| source | method       |  x1  | x1/2 | x1/4 | TL   | BR   | TL x1/2  | TL x1/4  |
| :----: | :----------: | :--: | :--: | :--: | :--: | :--: | :-----:  | :------: |
| RGB    | turbo_decoder| 15.7 | 14.4 | 11.6 | 15.6 | 15.8 | 16.3     | 16.1     |
| RGB    | cv::imdecode | 22.1 | 23.1 | 22.2 | 21.0 | 21.8 | 23.2     | 24.4     |
| Bayer  | turbo_decoder| 20.2 | :x:  | :x:  | 21.3 | 21.0 | :x:      | :x:      |
| Bayer  | cv::imdecode | 22.5 | :x:  | :x:  | 23.7 | 23.3 | :x:      | :x:      |

The tested RGB image is `2880x1860`. The tested Beyer image is `4096x2160`.

TL & BR stands for top-left cropping and bottom-right cropping.
