# JSBSim-Matlab
Matlab integration of JSBSim based on work of EliaTarasov

Compiled and tested on for Linux (Ubuntu). 

## Installation (JSBSim for MATLAB)
1. Type `git submodule init` and `git submodule update` - that will pull JSBSim submodule (note: consider updating to the latest JSBSIM from https://sourceforge.net/p/jsbsim/code/ci/master/tree/ ?)
2. Go to `JSBSim` directory ~~and make sure you are on `matlab_integration` branch~~
3. Configure JSBSim for shared libraries: `./autogen.sh --enable-libraries --disable-static --enable-shared`
4. Make JSBSim with shared libraries: `make` (it will take a moment)
5. Start Matlab (tested so far with 2014b) and navigate to the room directory of `JSBSim-Matlab`
6. In Matlab command line type: `mex ./JSBSimMatlab/MexJSBSim.cpp  ./JSBSimMatlab/JSBSimInterface.cpp -I./JSBSim/src -L./JSBSim/src/.libs -lJSBSim`
7. Now you can run for example `test1.m` or `test_c172_cruise_8K.m` or simply call set/get handles in your own script

## Installation (JSBSim for Simulink)
1. Do steps 1-5 as in the previous case
2. In Matlab command line type: `mex -v ./JSBSimMatlab/JSBSim_SFunction.cpp  ./JSBSimMatlab/JSBSimInterface.cpp -I./JSBSim/src -L./JSBSim/src/.libs -lJSBSim`
3. Now start `JSBSim_GUI.m` - it will open a GUI that you need to use
4. In the GUI first click on **Load Model** (it will load the Simulink model with JSBSim S-function block). **You have to do this even if your Simulink model is already loaded**
5. In the GUI type into the aircraft model name `c172x`
5. In the GUI click on **Initialize Model** It should output something like ```JSBSim Flight Dynamics Model v1.0 Sep  6 2016 17:16:47[JSBSim-ML v2.0]JSBSim startup beginning ...```
6. Now you can go to Simulink and run the simulation. You should get output like `JSBSim S-Function is initializing...` then lots of text and then `'c172x' Aircraft File has been successfully loaded! Simulation completed.` That means that the model was loaded properly into JSBSim engine.
7. you can examine `tout` and `xout` variables - `xout` has 12 states ([u-fps v-fps w-fps p-radsec q-radsec r-radsec h-sl-ft long-gc-deg lat-gc-deg phi-rad theta-rad psi-rad] and `tout` is simulation time. **NOTE: this is what I have so far - the output is still zero, looks like the simulation is not being stepped through**


