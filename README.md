This code implements the Incremental Reconstruction algorithm described in the paper 
Piazza, Enrico, Andrea Romanoni, and Matteo Matteucci. "Real-Time CPU-Based Large-Scale Three-Dimensional Mesh Reconstruction." IEEE Robotics and Automation Letters 3.3 (2018): 1584-1591.

Bibtex:

```
@article{piazza2018real,
  title={Real-Time CPU-Based Large-Scale Three-Dimensional Mesh Reconstruction},
  author={Piazza, Enrico and Romanoni, Andrea and Matteucci, Matteo},
  journal={IEEE Robotics and Automation Letters},
  volume={3},
  number={3},
  pages={1584--1591},
  year={2018},
  publisher={IEEE}
}
```


# DEPENDENCIES #
* opencv
* eigen3
* gmp
* mpfr
* CGAL
* boost

If you are on Ubuntu, you can install all the dependencies (execept CGAL) with: 
``` 
sudo apt install libopencv-dev libeigen3-dev libgmp-dev \
libmpfr-dev libmpfr4 libboost-all-dev
```
since CGAL is not up to date on ubuntu distro we advise to install it from the [sources](https://github.com/CGAL/cgal "CGAL").

# Generate runnable files
You can compile the sources using the simple cmake-make procedure.
First thing first, create the build directory:
```
mkdir build
```
Then enter the directory and generate the make file and the executables:
```
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```


Let notice, in the external directory the sources of `glm` and `rapidjson` are provided, therefore they do not need to be installed. However, if you already have those installed on your machine you may want to use your version. To do so:

1. open the `CMakeLists.txt` file
2. comment out lines `20` and `21`
3. add `find_library(GMP_LIBRARY gmp /usr/lib)` between line `9` and `10`.

# Run
Once the executable files have been generated you would find `sfmReconstructor` and `slamReconstructor` files into the `build` folder. To run one of them, e.g., `sfmReconstructor`, do the following:

1. go to the project main directory
2. create the output folder (you need to do this only once)
```
mkdir output
```
3. launch your file specifing the input data
```
sfmReconstructor data/sfm_data.json
```
