# JSBSim-Matlab
Matlab integration of JSBSim based on work of EliaTarasov

Compiled and tested on for Linux (Ubuntu). 

## Installation
1. Type `git submodule init` and `git submodule update` - that will pull JSBSim submodule (note: consider updating to the latest JSBSIM from https://sourceforge.net/p/jsbsim/code/ci/master/tree/ ?)
2. Go to `JSBSim` directory ~~and make sure you are on `matlab_integration` branch~~
3. Configure JSBSim for shared libraries: `./autogen.sh --enable-libraries --disable-static --enable-shared`
4. Make JSBSim with shared libraries: `make` (it will take a moment)
5. Start Matlab (tested so far with 2014b) and navigate to the root directory of `JSBSim-Matlab`
6. In Matlab command line type: `mex ./JSBSimMatlab/MexJSBSim.cpp  ./JSBSimMatlab/JSBSimInterface.cpp -I./JSBSim/src -L./JSBSim/src/.libs -lJSBSim`
