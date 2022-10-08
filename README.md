# MCL-Simulation
this Monte Carlo localization simulation source is Udacity Robotics Software Engineer Nanodegree, the code is the same code in `MCL_LAB` of Udacity ND but with some edits done by me. 

### prerequisites

* g++-5.5 OR above
* python3.8


### Compiling the Program
```bash
git clone https://github.com/khshmt/Monte_Carlo_localization_simulation.git
cd Monte_Carlo_localization_simulation/
make
```

### Running the Program
Before you run the program, make sure the `Images` directory is empty by running 

```bash
mkdir build
cd build
cmake ..
```
then run the executable file `app`

```bash
./build/bin/app
```
Wait for the program to iterate `50` times.

### Generated Images
After running the program, `50` images will be generated in the `Images` folder.

