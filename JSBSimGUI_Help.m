% To use the JSBSim S-Function GUI
% 
% 1. Select the simulink model that will be used by typing in the name of the simulink model.  The default model as 
% it would be entered into the textbox is "sfuntest_gui". Please take a look at the screenshot to get an idea of 
% how certain parameters need to be typed in.
% 2. Press the "Load Model" button to open the Simulink model. Any parameters saved in the Simulink model will be 
% displayed in the GUI. 
% 3. If the saved parameters are the parameters that are desired to run the model then simply run the simulation 
% as normal. If you would like to change the parameters then read on.
% 4. Change the aircraft model by typing its name into the ac name textbox. Be sure to include leading ' and terminating '
% characters.
% 5. Change the verbosity by clicking on a verbosity setting from the dropdown list. The new setting will be displayed in the
% textbox below it.
% 6. Change the simulation delta by typing it into the "Set JSBSim delta_T" textbox.  Do not use any ' characters here.
% 7. The "JSBSim Multiplier" changes the JSBSim/Simulink speed ratio.  For
% example by setting "10" as the JSBSim Multiplier value, JSBSim will
% complete 10 cycles for every Simulink cycle.  This allows for
% dramatically faster simulation runs.  However, keep in mind that JSBSim
% will only get new control inputs when a Simulink cycle has completed.
% 8. Change control and state initial parameters either by using the slider controls or using the texboxes.  
% The "Set Running" control is a toggle button and controls whether or not the engine(s) are set to a running state
% upon model initialization.
% 9. Once all the desired initialization parameters are set, press the "Initialize Model Parameters" button to change 
% the model parameters. You can verify the chages by looking at the S-Function's Function Block Parameters box.
% Also, the control input block's "initial value" parameters box are all set to
% the initial controls values.
% 10. Save the Simulink model if you want to keep the parameters.
% 11. After each simulation run, it is a good idea to press the "Reset" button on the GUI to clear any saved FCS 
% integrator states (if present) that may affect subsequent simulation runs.


