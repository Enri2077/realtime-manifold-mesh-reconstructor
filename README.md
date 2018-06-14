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
### OpenCV  (> 2.4.x) ###
sudo apt-get install libopencv-dev 

### Eigen3 (> 3.0.5) ###
sudo apt-get install libeigen3-dev

### gmp ###
sudo apt-get install libgmp-dev

### mpfr ###
sudo apt-get install libmpfr-dev

### CGAL (>4.3) ###
From the CGAL installation manual at http://doc.cgal.org/latest/Manual/installation.html :
* clone the CGAL git repository in a folder (FOLDER) or download the source code from the official page https://github.com/CGAL/cgal/releases
```
cd FOLDER
git clone https://github.com/CGAL/cgal.git CGAL-{CGAL_VERSION}
```
* configure CGAL
```
cd CGAL-{CGAL_VERSION}/
cmake .
```
* build the CGAL libraries
```
make
```
* install the libraries
```
sudo make install
```

At the time of writing {CGAL_VERSION} is 4.9

