# Animation3D for [Physically Based Animation class](http://www.ci.i.u-tokyo.ac.jp/~hachisuka/anim2018/)

## Install dependancy:
```
sudo apt-get install freeglut3 freeglut3-dev #OpenGL
sudo apt-get install libeigen3-dev #Eigen
sudo apt-get install libboost-all-dev #Boost
```

## Compile and Execute
### For assignment 3
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

## Demo
### Particle trajectory under gravity using Verlet Integration: [Video](https://www.youtube.com/watch?v=uvoZv55v3hE)
### Elastic collision: [Video](https://www.youtube.com/watch?v=i9KygzlNcBw)
### Constraint on a line segment: [Video](https://www.youtube.com/watch?v=kn-lZJEiL0w)
### Billiard simulation: [Video](https://www.youtube.com/watch?v=oEKaxwxVx-8)
### Gravitational field simulation: [Video](https://www.youtube.com/watch?v=9imJhutjM1Y)
### Gravitational field implementing Barnes-Hut algorithm: [Video](https://www.youtube.com/watch?v=_m35aLWKsHw)
### Newton cradle simulation without loosing kinetic energy: [Video](https://www.youtube.com/watch?v=Jx4CcILJVcQ)
### Rigid falling (with rotation) under gravity [Video](https://www.youtube.com/watch?v=0uAAKh6zzAw)
### Deformable solid falling under gravity [Video](https://www.youtube.com/watch?v=KSZJp0vm-6w)
### Billiards simulation with rigid cubes [Video](https://www.youtube.com/watch?v=wOM-8C0eqws)
### Wave simulation based on the shallow water equations 
* References: [Shallow water equations](https://perso.liris.cnrs.fr/alexandre.meyer/teaching/master_charanim/papers_pdf/coursenotes_shallowWater.pdf)
* [Video](https://www.youtube.com/watch?v=A6J8pfpZx8I)
### Fluids Simulation Using SPH(Smoothed Particle Hydrodynamics) and Dynamic Spatial Grid
* References: [Sph survival kit](https://www8.cs.umu.se/kurser/TDBD24/VT06/lectures/sphsurvivalkit.pdf), [Smoothed particle hydrodynamics fluid simulation](http://rlguy.com/sphfluidsim/)
* Demo with N = 1000 particles: [Video](https://www.youtube.com/watch?v=aHoQozNnp8M)
* Demo with N = 5000 particles: [Video](https://www.youtube.com/watch?v=sdG1NiOtwJY)
