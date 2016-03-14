# JSBSim-Matlab
Matlab integration of JSBSim based on work of EliaTarasov

Compiled and tested on for Linux (Ubuntu). 

## Installation
1. Go to `JSBSim` directory and make sure you are on `matlab_integration` branch
2. Configure for shared libraries: `./autogen.sh --enable-libraries --disable-static --enable-shared`
3. Make with shared libraries: `make`
4. Start Matlab and navigate to the room directory of `JSBSim-Matlab`
5. In Matlab command line type: `mex ./JSBSimMatlab/MexJSBSim.cpp  ./JSBSimMatlab/JSBSimInterface.cpp -I./JSBSim/src -L./JSBSim/src/.libs -lJSBSim`
