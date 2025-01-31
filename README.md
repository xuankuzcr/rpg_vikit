VIKIT
-----

Vikit (Vision-Kit) provides some tools for your vision/robotics project.
Far from stable.

A test for ros2 humble on Ubuntu22.04,and use Sophus 1.22.10.

vikit_common install not use ros,only make & sudo make install.

vikit_py not yet ported and tested.

vikit_ros install by colcon build.

### Sophus install

Sophus Installation for the 1.22.10 version.

```bash
git clone https://github.com/strasdat/Sophus.git -b 1.22.10
cd Sophus
mkdir build && cd build && cmake ..
make
sudo make install
```

### vikit_common install

```bash
e.g. mv ./vikit_common/  ~/vikit_common/ (a not ros dir)
cd ~/vikit_common/
mkdir build && cd build && cmake ..
make
sudo make install
```
### vikit_ros install

```bash
e.g. mv ./vikit_ros/  ~/ros2_ws/src/vikit_ros/ (a ros dir)
cd ~/ros2_ws/
colcon build --parallel-workers 1 --packages-select vikit_ros (only build vikit_ros package)
```
