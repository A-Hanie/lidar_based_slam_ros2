# lidar_based_slam_ros2

[![](https://mermaid.ink/img/pako:eNptk92O2jAQhV_F8jWgEMJfLloVAgu7kKWwe9E6qDKxAUuJTW1nKYt49xo7UYnUXMUz35zxGdtXmApCYQj3mTinRyw1WKwTDsz3DW30fR0T_UtlOAexIbeg2fwCRmhDNZhzphnOwEoounU1I5seo02xU6lkOwq0MNyp0GCciYIAzMn_6sa2LkJrmlL2QU2K8aokwhqXWGSxCXqTmKu9kHkNNK1GWFEwlTivhCe2YoqmLNNUOlyB3QVETGnM04qbWu4JReLMFc5PWW0PJfRkodl1iU-VCfZJydebS89M2gzJQnP0DwB3_sz0EbhdUFJTnduCZzQWXDNemMZSpFQpxg_bB-EfVDnQxZ7t4sVM7GCcGGsxPdem8X4XAHH0Voq82IIFej8RrKk9iJEUmKRY6cejWFhuWXErrI9lZmkzMVoVu4ypI3AEudsridgSr2hm1DPn20GqBF4tsLouhawNuBrh6tFp9BArx_odTTipaa3RBn-4VoKDNf1dUKVLYu26JRw2YE5ljhkxN_16TyZQH2lOExiaX0L3uMh0AhN-MygutNhceApDLQvagIW1EDF8MBcLhnucKRM9Yf5TiNoahlf4B4a-77fa7aAf-J7X7vSCQbcBLzBsBoNW0Ot3u8Nup9sLvOHw1oCfVqLd8tte3-8Mg0G_0297XtCAlDAt5NK9TvtIb38BBQgd6Q?type=png)](https://mermaid.live/edit#pako:eNptk92O2jAQhV_F8jWgEMJfLloVAgu7kKWwe9E6qDKxAUuJTW1nKYt49xo7UYnUXMUz35zxGdtXmApCYQj3mTinRyw1WKwTDsz3DW30fR0T_UtlOAexIbeg2fwCRmhDNZhzphnOwEoounU1I5seo02xU6lkOwq0MNyp0GCciYIAzMn_6sa2LkJrmlL2QU2K8aokwhqXWGSxCXqTmKu9kHkNNK1GWFEwlTivhCe2YoqmLNNUOlyB3QVETGnM04qbWu4JReLMFc5PWW0PJfRkodl1iU-VCfZJydebS89M2gzJQnP0DwB3_sz0EbhdUFJTnduCZzQWXDNemMZSpFQpxg_bB-EfVDnQxZ7t4sVM7GCcGGsxPdem8X4XAHH0Voq82IIFej8RrKk9iJEUmKRY6cejWFhuWXErrI9lZmkzMVoVu4ypI3AEudsridgSr2hm1DPn20GqBF4tsLouhawNuBrh6tFp9BArx_odTTipaa3RBn-4VoKDNf1dUKVLYu26JRw2YE5ljhkxN_16TyZQH2lOExiaX0L3uMh0AhN-MygutNhceApDLQvagIW1EDF8MBcLhnucKRM9Yf5TiNoahlf4B4a-77fa7aAf-J7X7vSCQbcBLzBsBoNW0Ot3u8Nup9sLvOHw1oCfVqLd8tte3-8Mg0G_0297XtCAlDAt5NK9TvtIb38BBQgd6Q)

##  Installation

#### 1. copy the repo and place it inside src then build with

```bash
cd ~/ros2_ws/src
git clone https://github.com/A-Hanie/lidar_based_slam_ros2.git
```
#### 2. Build the Workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```
## Running SLAM

#### 1. Prepare Lidar Data:

Place your Lidar data inside the `~/data/data` directory to ensure it can be accessed by point_cloud_reader node.

#### 2. Launch the Point Cloud Reader:

```bash
ros2 run point_cloud_reader point_cloud_reader.py
```

#### 3. Run SLAM:

```bash
ros2 launch ndt_slam ndt_slam.launch.py
```


## Map saving

#### Save the Map:

Save the map by using the following ROS2 service call:
```bash
ros2 service call /save_map std_srvs/srv/Trigger "{}"
```

