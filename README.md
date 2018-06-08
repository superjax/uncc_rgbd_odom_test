# UNCC RGBD Odometry Test Executable

This is a simple example program that computes the transform between two RGBD image pairs using the [uncc_rgbd_odom](https://github.com/uncc-visionlab/uncc_rgbd_odom/tree/fadeada9f19acdd9b8925cf1b96094ce1e134e9b) package.

The images were taken on a small RC car with an Intel Realsense D430 RGBD Camera, and in the [images](images) folder.  PNG images are limited to 8-bit resolution, so the binary image format is used to encode the full resolution image.


## Installation
The UNCC RGBD Odometry package is included as submodule, pull it in, and use CMake (CMake will get mad at you if you are missing packages)

```
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make -j
```

## Running the example
```
cd build
./test_program
```




