## Animation3D for [Physically Based Animation class](http://www.ci.i.u-tokyo.ac.jp/~hachisuka/anim2018/)

### Install dependancy:
```
sudo apt-get install freeglut3 freeglut3-dev #OpenGL
sudo apt-get install libeigen3-dev #Eigen
sudo apt-get install libboost-all-dev #Boost
```

### Compile and Execute
#### For assignment 3
```
# Compile
mkdir build_assign3
cp CMakeLists.txt.assign3 CMakeLists.txt
cd build_assign3
cmake ..
make

# Execute
./water_demo
```
The same method can be used to compile and execute for assignment 1 and assignment 2
