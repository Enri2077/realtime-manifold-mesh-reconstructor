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

