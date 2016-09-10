/* JSBSim SFunction GUI version 0.3
 * The state vector has been simplified to 12 states instead of 19.  This has been done as a result
 * of needing to simplify the state vector for the upcoming trim and linearization functions
 * The calculated outputs vector has now been expanded to show some of the data missing from the state
 * vector, with the exception of the quaternoins, which may be added later.
 * The new GUI allows convenient control of all
 * initialization parameters using both sliders and text boxes.  The
 * simulink model, aircraft model, verbosity, JSBSim delta time, initial controls,
 * and initial states can all be controlled through the GUI.  
 * A JSBSim Multiplier function has been added to increase the execution rate of the simulation.
 * This allows JSBSim to complete from 1 to 100 simulation cycles for every Simulink cycle.
 * **************************************************************************************************
 * Bug fixes
 * %%% Fixed issues with Debug Verbosity settings
 * %%% Fixed problem with "verbose" Verbosity setting that did not allow simulation to run properly
 * %%% Fixed issue with throttles not being initialized properly and angines not being properly spooled up to the 
 * intended power setting
 *
 * 01/22/10 Brian Mills
 * 
 * *********Discrete States Version******************************************************************
 * JSBSim calculates states.  NO integration performed by Simulink.
 * Use fixed step discrete state solver
 * Basic implementation of a JSBSim S-Function that takes 5 input parameters
 * at the S-Function's block parameters dialog box:
 * 'ac_name_string', 
 * [u-fps v-fps w-fps p-radsec q-radsec r-radsec h-sl-ft long-gc-deg lat-gc-deg 
 *   phi-rad theta-rad psi-rad],
 * [throttle-cmd-norm aileron-cmd-norm elevator-cmd-norm rudder-cmd-norm mixture-cmd-norm set-running flaps-cmd-norm gear-cmd-norm],
 * [delta_T], 'verbosity'
 * Verbosity can either be set to 'Silent', 'Verbose', 'VeryVerbose' or 'Debug'
 * The model currently takes 8 control inputs:throttle, aileron, elevator, rudder, mixture, set-running, flaps and gear.
 * The model has 12 states:[u-fps v-fps w-fps p-rad-sec q-rad-sec r-rad-sec h-sl-ft long-deg lat-deg phi-rad theta-rad psi-rad] 
 * Model has 4 output ports: state vector, control output vector, propulsion output vector and calculated output vector.
 * States output [u-fps v-fps w-fps p-rad-sec q-rad-sec r-rad-sec h-sl-ft long-deg lat-deg phi-rad theta-rad psi-rad] 
 * Flight Controls output [thr-pos-norm left-ail-pos-rad el-pos-rad tvc-pos-rad rud-pos-rad flap-pos-norm right-ail-pos-rad 
 * speedbrake-pos-rad spoiler-pos-rad lef-pos-rad gear-pos-norm Nose-gear-steering-pos-deg gear-unit-WOW]
 * Propulsion output piston (per engine) [prop-rpm prop-thrust-lbs mixture fuel-flow-gph advance-ratio power-hp pt-lbs_sqft 
 * volumetric-efficiency bsfc-lbs_hphr prop-torque blade-angle prop-pitch]
 * Propulsion output turbine (per engine) [thrust-lbs n1 n2 fuel-flow-pph fuel-flow-pps pt-lbs_sqft pitch-rad reverser-rad yaw-rad inject-cmd 
 * set-running fuel-dump]
 * Calculated outputs [pilot-Nz alpha-rad alpha-dot-rad-sec beta-rad beta-dot-rad-sec vc-fps vc-kts 
 *						Vt-fps vg-fps mach climb-rate-fps]
 *
 * The UpdateStates method added to JSBSimInterface is called for every s-function simulation time step.
 * Currently it is advised that if the AC model FCS has integrators, then after each simulation run "clearSF" 
 * should be entered at the Matlab command line to reset the simulation.
 * This will ensure that every consecutive simulation run starts from the same initial states.  
 * It is planned to fix this in the near future.
 * Please look in the mdlInitializeSizes method for more detailed input port and output port details.
 * *************************************************************************************************************************
 * *************************************************************************************************************************
 * 08/08/09 JSBSimSFunction revision 1.0 for campatibility with JSBSim 1.0
 * Brian Mills
 * *************************************************************************************************************************
 * JSBSimInterface written by Agostino De Marco for use in the JSBSimMexFunction project. Additional functions have been added and changes 
 * made to work with SFunction API. Thanks to Agostino for providing the basis for this project.
 * *************************************************************************************************************************

*/
#ifdef __cplusplus
 
#endif       // defined within this scope
#define S_FUNCTION_NAME  JSBSim_SFunction
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include "mex.h"
#include "mclcppclass.h"
#include "matrix.h"
#include <iostream>
#include <string>
#include <vector>
#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <models/FGFCS.h>
#include "JSBSimInterface.h"


// 12 States of Initial Condition Vector
#define u_fps			mxGetPr(ssGetSFcnParam(S, 1))[0]
#define v_fps			mxGetPr(ssGetSFcnParam(S, 1))[1]
#define w_fps			mxGetPr(ssGetSFcnParam(S, 1))[2]

#define p_radsec          mxGetPr(ssGetSFcnParam(S, 1))[3]
#define q_radsec          mxGetPr(ssGetSFcnParam(S, 1))[4]
#define r_radsec          mxGetPr(ssGetSFcnParam(S, 1))[5]

#define h_sl_ft				mxGetPr(ssGetSFcnParam(S, 1))[6]
#define long_gc_deg			mxGetPr(ssGetSFcnParam(S, 1))[7]
#define lat_gc_deg			mxGetPr(ssGetSFcnParam(S, 1))[8]

#define phi_rad			mxGetPr(ssGetSFcnParam(S, 1))[9]
#define theta_rad		mxGetPr(ssGetSFcnParam(S, 1))[10]
#define psi_rad			mxGetPr(ssGetSFcnParam(S, 1))[11]


// Initial conditions for the controls 
#define throttle         mxGetPr(ssGetSFcnParam(S, 2))[0]
#define aileron              mxGetPr(ssGetSFcnParam(S, 2))[1]
#define elevator              mxGetPr(ssGetSFcnParam(S, 2))[2]
#define rudder              mxGetPr(ssGetSFcnParam(S, 2))[3]
#define mixture             mxGetPr(ssGetSFcnParam(S, 2))[4]
#define runset              mxGetPr(ssGetSFcnParam(S, 2))[5]
#define flaps              mxGetPr(ssGetSFcnParam(S, 2))[6]
#define gear              mxGetPr(ssGetSFcnParam(S, 2))[7]

#define delta_t				mxGetPr(ssGetSFcnParam(S, 3))[0] // delta T
#define verbosity			ssGetSFcnParam(S, 4) //Verbosity parameter
#define ac_name				ssGetSFcnParam(S, 0) //Name of JSBSim aircraft model file to load
#define multiplier			mxGetPr(ssGetSFcnParam(S, 5))[0] //JSBSim multiplier

// Necessary to create mxArray of initial conditions 
#define NUMBER_OF_STRUCTS (sizeof(ic)/sizeof(struct init_cond))
#define NUMBER_OF_FIELDS (sizeof(field_names)/sizeof(*field_names))

JSBSim::FGFDMExec FDMExec;	// Instantiate new JSBSim FDMExec object

struct init_cond
		{
			const char *name;
			double value;
		};
struct verbosity_level
{
	const char *v_name;
};

/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
  /* See sfuntmpl_doc.c for more details on the macros below */
    ssSetNumSFcnParams(S, 6);  /* Number of expected parameter vectors*/
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    //ssSetNumContStates(S, 12);
    ssSetNumDiscStates(S, 12);

    /* if (!ssSetNumInputPorts(S, 1)) return; */
	ssSetNumInputPorts(S, 1);
    ssSetInputPortWidth(S, 0, 8);//[thr ail el rud mxtr run flap gear]
    /* ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */
     /* ssSetInputPortDirectFeedThrough(S, 0, 1); */

    if (!ssSetNumOutputPorts(S, 4)) return;
    ssSetOutputPortWidth(S, 0, 12);//The model has 12 states:[u v w p q r h-sl-ft long lat phi theta psi]	
	
	/* Flight Controls output [thr-pos-norm left-ail-pos-rad el-pos-rad tvc-pos-rad rud-pos-rad flap-pos-norm right-ail-pos-rad 
	 * speedbrake-pos-rad spoiler-pos-rad lef-pos-rad gear-pos-norm Nose-gear-steering-pos-deg gear-unit-WOW]
	 */
	ssSetOutputPortWidth(S, 1, 13);

	/* Propulsion output piston (per engine) [prop-rpm prop-thrust-lbs mixture fuel-flow-gph advance-ratio power-hp pt-lbs_sqft 
	 * volumetric-efficiency bsfc-lbs_hphr prop-torque blade-angle prop-pitch]
	 * Propulsion output turbine (per engine) [thrust-lbs n1 n2 fuel-flow-pph fuel-flow-pps pt-lbs_sqft pitch-rad reverser-rad yaw-rad inject-cmd 
	 * set-running fuel-dump]
	 */
	ssSetOutputPortWidth(S, 2, 48);
		

	ssSetOutputPortWidth(S, 3, 11);//Calculated outputs [pilot-Nz alpha alpha-dot beta beta-dot vc-fps vc-kts 
 						           //					 Vt-fps vg-fps mach climb-rate]    
	//ssSetOutputPortWidth(S, 4, 12);//JSBSim Calculated States output [u v w p q r q1 q2 q3 q4 long-deg lat-deg z-ft phi theta psi h-ft alpha beta]

	//ssSetNumSampleTimes(S, 1);
    if(!ssSetNumDWork(   S, 6)) return;

    ssSetDWorkWidth(     S, 0, ssGetInputPortWidth(S,0));//Work vector for input port
    ssSetDWorkDataType(  S, 0, SS_DOUBLE); /* use SS_DOUBLE if needed */

    ssSetDWorkWidth(     S, 1, ssGetNumDiscStates(S));//Work vector for states * may need to add actuator states!
    ssSetDWorkDataType(  S, 1, SS_DOUBLE);

	ssSetDWorkWidth(     S, 2, ssGetNumDiscStates(S));	//Work vector derivatives
    ssSetDWorkDataType(  S, 2, SS_DOUBLE);

	ssSetDWorkWidth(     S, 3, ssGetOutputPortWidth(S,1));//Work vector for flight controls outputs
    ssSetDWorkDataType(  S, 3, SS_DOUBLE);

	ssSetDWorkWidth(     S, 4, ssGetOutputPortWidth(S,2));//Work vector for propulsion outputs
    ssSetDWorkDataType(  S, 4, SS_DOUBLE);

	ssSetDWorkWidth(     S, 5, ssGetOutputPortWidth(S,3));//Work vector for calculated outputs
    ssSetDWorkDataType(  S, 5, SS_DOUBLE);	


	ssSetNumPWork(S, 1); // reserve element in the pointers vector
                         // to store a C++ object

    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, 0);
		
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {	
	    /* Create new JSBSimInterface object and initialize it with delta_t and
		   also create a pointer to the JSBSimInterface object so we can access its member
		   functions.
		*/
	    ssGetPWork(S)[0] = (void *) new JSBSimInterface(&FDMExec, delta_t);// IC parameter 4 passed here!
		JSBSimInterface *JII = (JSBSimInterface *) ssGetPWork(S)[0];   // retrieve C++ object pointers vector
		//*********************************************************************************************//
		/* create an mxStructureArray to set the verbosity */
	  
		const char *v_field_names[] = {"v_name"};

		char v_buf[128];
		mwSize v_buflen;
		v_buflen = mxGetNumberOfElements(verbosity) + 1;
		mxGetString(verbosity, v_buf, v_buflen);
		//string s_verbosity = "";
		//s_verbosity = string(v_buf);

		struct verbosity_level v[] = {v_buf};
		mwSize dims1[1] = {1};
		int v_field;
		//mwIndex j;
		mxArray *prhs[2];//create mxArray of size 2
		prhs[1] = mxCreateStructArray(1, dims1, 1, v_field_names);
		//name_field = mxGetFieldNumber(prhs[0],"name");
        v_field = mxGetFieldNumber(prhs[1],"v_name");
		//for (i=0; i<NUMBER_OF_STRUCTS; i++) {
			//mxArray *field_value1;
			
			//mxSetFieldByNumber(prhs[0],i,name_field,mxCreateString(ic[i].name));
			//field_value1 = mxCreateDoubleMatrix(1,1,mxREAL);
			//*mxGetPr(field_value1) = v[0].v_name;
			
			mxSetFieldByNumber(prhs[1],0,v_field,mxCreateString(v[0].v_name));
			mexPrintf("\n%s Verbosity level requested by S-Function.\n", v[0].v_name);
 /***************************************************************************************/
		 //mexPrintf("Before JI->Init.\n");
		JII->SetVerbosity(prhs[1]); // Need to create an mxArray structure prhs1 for the verbosity level
		// mxDestroyArray(*prhs);
	    mexPrintf("\n");
	    mexPrintf("JSBSim S-Function is initializing...\n");
		mexPrintf("\n");
		mexPrintf("Note: For Aircraft with integrators in the FCS, please type 'clearSF' to completely reset S-Function.\n");
		mexPrintf("\n");
		char buf[128];
		mwSize buflen;
		buflen = mxGetNumberOfElements(ac_name) + 1;
		mxGetString(ac_name, buf, buflen);
		string aircraft = "";
		aircraft = string(buf);
		if(!JII->Open(aircraft))
		{
			mexPrintf("Aircraft file could not be loaded.\n");
			mexPrintf("\n");
		}
		
		else
			mexPrintf("'%s' Aircraft File has been successfully loaded!\n", aircraft.c_str());
	
			//PrintCatalog(); Print AC Catalog when ac model loads
		/*
	    real_T *x = ssGetRealDiscStates(S); 		
		int j;
        // Initialize the state vector 
        for (j = 0; j < ssGetNumDiscStates(S); j++)
        {
            x[j] = mxGetPr(ssGetSFcnParam(S, 1))[j];
		}
		*/

		

		/* create an mxStructureArray to recreate the initial conditions in test_1.m */
		const char *field_names[] = {"name", "value"};
		struct init_cond ic[] = {{"u-fps", u_fps},{"v-fps", v_fps},{"w-fps", w_fps},{"p-rad_sec", p_radsec},{"q-rad_sec", q_radsec},{"r-rad_sec", r_radsec},
		{"h-sl-ft", h_sl_ft},{"long-gc-deg", long_gc_deg},{"lat-gc-deg", lat_gc_deg},{"phi-rad", phi_rad},{"theta-rad", theta_rad},{"psi-rad", psi_rad},
		{"fcs/throttle-cmd-norm", throttle}, {"aileron-cmd-norm", aileron}, {"elevator-cmd-norm", elevator}, {"rudder-cmd-norm", rudder}, 
		{"fcs/mixture-cmd-norm", mixture}, {"set-running", runset}, {"flaps-cmd-norm", flaps}, {"gear-cmd-norm", gear}, {"multiplier", multiplier}};
		mwSize dims[2] = {1,NUMBER_OF_STRUCTS};
		int name_field, value_field;
		mwIndex i;
		//mxArray *prhs[1];
		prhs[0] = mxCreateStructArray(2, dims, NUMBER_OF_FIELDS, field_names);
		name_field = mxGetFieldNumber(prhs[0],"name");
        value_field = mxGetFieldNumber(prhs[0],"value");
		for (i=0; i<NUMBER_OF_STRUCTS; i++) {
			mxArray *field_value;
			/* Use mxSetFieldByNumber instead of mxSetField for efficiency
			mxSetField(plhs[0],i,"name",mxCreateString(friends[i].name); */
			mxSetFieldByNumber(prhs[0],i,name_field,mxCreateString(ic[i].name));
			field_value = mxCreateDoubleMatrix(1,1,mxREAL);
			*mxGetPr(field_value) = ic[i].value;
			/* Use mxSetFieldByNumber instead of mxSetField for efficiency
			mxSetField(plhs[0],i,"name",mxCreateString(friends[i].name); */
			mxSetFieldByNumber(prhs[0],i,value_field,field_value);
			}
			JII->Init(prhs[0]);
		 //mexPrintf("After JI->Init.\n");		 
	  
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
	   
	  real_T *x2 = ssGetRealDiscStates(S);

		x2[0] = u_fps;
		x2[1] = v_fps;
		x2[2] = w_fps;
		x2[3] = p_radsec;
		x2[4] = q_radsec;
		x2[5] = r_radsec;
		x2[6] = h_sl_ft;
		x2[7] = long_gc_deg;
		x2[8] = lat_gc_deg;
		x2[9] = phi_rad;
		x2[10] = theta_rad;
		x2[11] = psi_rad;
		//x[12] = alpha_rad;
		//x[13] = beta_rad;
		
  }
#endif /*  MDL_START */

#undef MDL_SET_OUTPUT_PORT_WIDTH   /* Change to #undef to remove function */
#if defined(MDL_SET_OUTPUT_PORT_WIDTH) && defined(MATLAB_MEX_FILE)
  /* Function: mdlSetOutputPortWidth ==========================================
   * Abstract:
   *    This method is called with the candidate width for a dynamically
   *    sized port.  If the proposed width is acceptable, the method should
   *    go ahead and set the actual port width using ssSetOutputPortWidth.  If
   *    the size is unacceptable an error should generated via
   *    ssSetErrorStatus.  Note that any other dynamically sized input or
   *    output ports whose widths are implicitly defined by virtue of knowing
   *    the width of the given port can also have their widths set via calls
   *    to ssSetInputPortWidth or ssSetOutputPortWidth.
   */
  static void mdlSetOutputPortWidth(SimStruct *S, int portIndex, int width)
  {
	 ssSetOutputPortWidth(S, 2, ssGetDWorkWidth(S,4));
	 

  } /* end mdlSetOutputPortWidth */
#endif /* MDL_SET_OUTPUT_PORT_WIDTH */

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
	//real_T *x = ssGetContStates(S);
    real_T *x2 = ssGetRealDiscStates(S);  
    real_T *y1 = ssGetOutputPortRealSignal(S, 0);
	real_T *y2 = ssGetOutputPortRealSignal(S, 1);
	real_T *y3 = ssGetOutputPortRealSignal(S, 2);
	real_T *y4 = ssGetOutputPortRealSignal(S, 3);
	//real_T *y5 = ssGetOutputPortRealSignal(S, 4);
	double *w3 =  (double *) ssGetDWork(S,3);
	double *w4 = (double *) ssGetDWork(S,4);
	double *w5 = (double *) ssGetDWork(S,5);
    int i;
/*
	for (i = 0; i < ssGetNumContStates(S); i++)
	 {
		y1[i] = x[i]; // outputs are the states 
	 }
*/
	for (i = 0; i < ssGetNumDiscStates(S); i++)
	 {
		y1[i] = x2[i]; /* outputs are the states */
	 }

	for (i = 0; i < ssGetDWorkWidth(S,3); i++)
	 {
		y2[i] = w3[i]; // outputs are the flight control outputs 
	 }

	for (i = 0; i < ssGetDWorkWidth(S,4); i++)
	 {
		y3[i] = w4[i]; // outputs are the propulsion outputs 
	 }
	for (i = 0; i < ssGetDWorkWidth(S,5); i++)
	 {
		y4[i] = w5[i]; // outputs are the calculated outputs 
	 }
	
}



#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
		/* send update inputs to JSBSimInterface, run one cycle, 
	   retrieve state vector, and update sim state vector 
	  */
	  //mexPrintf("Before JII pointer object creation.\n");
	 JSBSimInterface *JII = (JSBSimInterface *) ssGetPWork(S)[0];   // retrieve C++ object pointers vector
	 //mexPrintf("After JII pointer creation.\n");
	 real_T *x2 = ssGetRealDiscStates(S);
	 //real_T *x = ssGetContStates(S);
	 InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
	 //double *derivatives = (double *) ssGetDWork(S,2);
	 double *inputs   = (double *) ssGetDWork(S,0);
	 double *states = (double *) ssGetDWork(S,1);
	 double *controls = (double *) ssGetDWork(S,3);
	 double *propulsion = (double *) ssGetDWork(S,4);
	 double *outputs = (double *) ssGetDWork(S,5);
	 int k;
	 for (k=0; k < ssGetDWorkWidth(S,0); k++) {
        inputs[k] = (*uPtrs[k]);
     }
	 for (k=0; k < ssGetDWorkWidth(S,1); k++) {
        states[k] = x2[k];
     }
	 /*
	 mexPrintf("Before JII->UpdateStates.\n");
	 //If integrator override flag is 1, then integrate in Simulink, else integrate in JSBSim
	 
	 if(integrator == 1)
	  {
	   JII->UpdateStates(inputs, derivatives, states, controls, propulsion, outputs); // call to JSBSimInterface to get updated states from JSBSim 
	   }
	  else
		 
	 JII->UpdateStates(inputs, states, controls, propulsion, outputs); // call to JSBSimInterface to get updated states from JSBSim  
	 //mexPrintf("After JII->UpdateStates.\n");
	 */
	for (k=0; k < ssGetDWorkWidth(S,1); k++) {
        x2[k] = states[k];
    } 
	/* for (k=0; k < ssGetDWorkWidth(S,2); k++) {
        dx[k] = derivatives[k];
    }*/
	//UNUSED_ARG(tid);
  }
#endif /* MDL_UPDATE */



#undef MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
	  real_T *dx = ssGetdX(S);
	  double *w2 =  (double *) ssGetDWork(S,2);
	  for (int i = 0; i < ssGetDWorkWidth(S,2); i++)
	 {
		dx[i] = w2[i]; // outputs are the flight control outputs 
	 }

  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
	
	//JSBSimInterface *JII = (JSBSimInterface *) ssGetPWork(S)[0];   // retrieve C++ object pointers vector
	//JII->ResetToInitialCondition();
	//JII->~JSBSimInterface();
	mexPrintf("\n");
	mexPrintf("Simulation completed.\n");
	mexPrintf("\n");
	
}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
