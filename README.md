OCLSLAM
=======

`OCLSLAM` is an implementation of a simultaneous localization and mapping (SLAM) algorithm running on a GPU and written in `OpenCL`. The implemented pipeline relies on the random ball cover (`RBC`) data structure and `ICP` algorithm to perform efficiently the 3D point cloud registration. The `OctoMap` framework is utilized in order to build a map and access it for localization, navigation, and exploration purposes.

![oclslam](https://github.com/nlamprian/OCLSLAM/wiki/assets/oclslam.png)

The project is under development, but there are already some rough results (see above). The system is able to register a point cloud and update a map of `10cm` resolution at around `10Hz`, with the CPU staying mainly unaffected.

Note
----

The project is being developed and tested on `Ubuntu 14.04.2`, on a system with an `AMD R9 270X` GPU.

The complete `documentation` is available [here](https://oclslam.nlamprian.me).

For more details on the implemented algorithms, take a look at the project's [wiki](https://github.com/nlamprian/OCLSLAM/wiki/Algorithms).

Dependencies
------------

The project has dependencies on the [libfreenect](https://github.com/OpenKinect/libfreenect/), [CLUtils](https://github.com/nlamprian/CLUtils), [GuidedFilter](https://github.com/nlamprian/GuidedFilter), [RandomBallCover](https://github.com/nlamprian/RandomBallCover), [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), [ICP](https://github.com/nlamprian/ICP), and [OctoMap](https://github.com/OctoMap/octomap) libraries.

All these dependencies (apart from libfreenect) are automatically downloaded by cmake, if they are not available on your system. Please note that you'll have to [configure Mercurial](http://eigen.tuxfamily.org/index.php?title=Mercurial) before Eigen is downloaded.

Examples
--------

There are three applications. Two of them were developed as I was experimenting with the octomap API, and they demonstrate how to integrate a point cloud in an `OcTree` and `ColorOcTree` data structure. The third application implements the SLAM system. It builds a map, uses OpenGL to visualize the progress, and can store the map on disk.

Compilation
-----------

```bash
https://github.com/nlamprian/OCLSLAM.git
cd OCLSLAM

mkdir build
cd build

cmake -DBUILD_EXAMPLES=ON ..
# or to build the tests too
cmake -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..

make

# to run the examples (from the build directory!)
./bin/oclslam_octree_example
./bin/oclslam_coloroctree_example
./bin/oclslam_slam

# to run the tests
./bin/oclslam_tests_oclslam
# or with profiling information
./bin/oclslam_tests_oclslam --profiling

# to install the libraries
sudo make install
# you'll need to copy manually the kernel 
# files into your own projects

# to build the docs
make doxygen
firefox docs/html/index.html
```
