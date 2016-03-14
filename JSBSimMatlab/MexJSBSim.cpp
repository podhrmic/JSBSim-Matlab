// MexJSBSim.cpp : Defines the entry point for the console application.
// by A. Bryant Nichols Jr. 2007

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

using namespace std;

// function prototypes
// ...

// this object should be persistent in memory until
// the MEX-function is cleared by Matlab
JSBSim::FGFDMExec FDMExec;
JSBSimInterface JI(&FDMExec);

void helpOptions() {
	mexPrintf("function usage:\n"                                           );
	mexPrintf("result = MexJSBSim( string_directive [, string, value] )\n"  );
	mexPrintf("\n"                                                          );
	mexPrintf("Examples:\n"                                                 );
	mexPrintf("    res = MexJSBSim('help');\n"                              );
	mexPrintf("       Prints this help text. returns 1 (always)\n\n"        );
	mexPrintf("    res = MexJSBSim('open','c172r');\n"                      );
	mexPrintf("       returns 1 if success, 0 otherwise\n\n"                );
	mexPrintf("    res = MexJSBSim('SetVerbosity',<verbosity level>)\n"     );
	mexPrintf("       <verbosity level> = 'silent' or 0,\n"                 );
	mexPrintf("                           'verbose' or 1,\n"                );
	mexPrintf("                           'very verbose' or 2,\n"           );
	mexPrintf("       returns 1 if success, 0 otherwise\n\n"                );
	mexPrintf("    res = MexJSBSim('get','fcs/elevator-cmd-norm')\n"        );
	mexPrintf("       returns the value of the property,\n"                 );
	mexPrintf("       or -9999 if property not found\n\n"                   );
	mexPrintf("    res = MexJSBSim('set','fcs/elevator-cmd-norm',-0.5)\n"   );
	mexPrintf("       returns 1 if success, 0 otherwise\n\n"                );
	mexPrintf("    res = MexJSBSim('set','elevator-cmd-norm',-0.5)\n"       );
	mexPrintf("    res = MexJSBSim('set','u-fps',80.5)\n"                   );
	mexPrintf("       same as above, but it scans a set of predefined,\n"   );
	mexPrintf("       and commonly used variable names:\n"                  );
	mexPrintf("            'u-fps', 'v-fps', 'w-fps'\n"                     );
	mexPrintf("            'p-rad_sec', 'q-rad_sec', 'r-rad_sec'\n"         );
	mexPrintf("            'h-sl-ft'\n"                                     );
	mexPrintf("            'long-gc-deg'\n"                                 );
	mexPrintf("            'lat-gc-deg'\n"                                  );
	mexPrintf("            'phi-rad', 'theta-rad', 'psi-rad'\n"             );
	mexPrintf("            'elevator-cmd-norm'\n"                           );
	mexPrintf("            'aileron-cmd-norm'\n"                            );
	mexPrintf("            'rudder-cmd-norm'\n\n"                           );
	mexPrintf("    res = MexJSBSim('init',ic)\n"                            );
	mexPrintf("       initializes the JSBSim::FGFDMExec by means of the\n"  );
	mexPrintf("       matlab structure ic.\n"                                        );
	mexPrintf("       The structure ic must have the two fields: name and value.\n"  );
	mexPrintf("       The Matlab user is forced to build such a structure first,\n"  );
	mexPrintf("       then to pass it to the mex-file. Example:\n"                   );
	mexPrintf("       >> ic(1).name = 'u-fps'; ic(1).value = 80;\n"                  );
	mexPrintf("       >> ic(2).name = 'v-fps'; ic(2).value =  0;\n"                  );
	mexPrintf("       >> ic(3).name = 'w-fps'; ic(3).value =  0;\n"                  );
	mexPrintf("       >> ic(4).name = 'p-rad_sec'; ic(4).value =  0;\n"              );
	mexPrintf("       >> MexJSBSim('init', ic);\n\n"                                 );
	mexPrintf("       returns 1 if success, 0 otherwise\n\n"                         );
	mexPrintf("    res = MexJSBSim('dot',ic)\n"                                      );
	mexPrintf("       same as above, regarding input parameters.\n"                  );
	mexPrintf("       returns a res vector containing the following time rates.\n"   );
	mexPrintf("       ___ body-axis velocity component rates\n");
	mexPrintf("       res( 1) : u_dot (ft/s/s)\n");
	mexPrintf("       res( 2) : v_dot (ft/s/s)\n");
	mexPrintf("       res( 3) : w_dot (ft/s/s)\n");
	mexPrintf("       ___ body-axis angular velocity component rates\n");
	mexPrintf("       res( 4) : p_dot (rad/s/s)\n");
	mexPrintf("       res( 5) : q_dot (rad/s/s)\n");
	mexPrintf("       res( 6) : r_dot (rad/s/s)\n");
	mexPrintf("       ___ orientation quaternion component rates\n");
	mexPrintf("       res( 7) : q1_dot \n");
	mexPrintf("       res( 8) : q2_dot \n");
	mexPrintf("       res( 9) : q3_dot \n");
	mexPrintf("       res(10) : q4_dot \n");
	mexPrintf("       ___ ECEF position vector component rates\n");
	mexPrintf("       res(11) : x_dot (ft/s)\n");
	mexPrintf("       res(12) : y_dot (ft/s)\n");
	mexPrintf("       res(13) : z_dot (ft/s)\n");
	mexPrintf("       ___ Euler angle rates\n");
	mexPrintf("       res(14) : phi_dot   (rad/s)\n");
	mexPrintf("       res(15) : theta_dot (rad/s)\n");
	mexPrintf("       res(16) : psi_dot   (rad/s)\n");
	mexPrintf("       ___ Altitude rate\n");
	mexPrintf("       res(17) : h_dot     (ft/s)\n" );
	mexPrintf("       ___ Aerodynamic angle rates\n");
	mexPrintf("       res(18) : alpha_dot (rad/s)\n");
	mexPrintf("       res(19) : beta_dot  (rad/s)\n");
}

// the gataway function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

	string aircraftName = "";
	string option = "";

	if (nrhs>0)
	{
		char buf[128];
		mwSize buflen;
		buflen = mxGetNumberOfElements(prhs[0]) + 1;
		mxGetString(prhs[0], buf, buflen);

		if (nrhs==1)
		{
			option = string(buf);

			if ( option == "help")
			{
				helpOptions();
				plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
				*mxGetPr(plhs[0]) = 1;
			}
			else
			{
				if ( option == "catalog")
				{
					JI.PrintCatalog();
					plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
					*mxGetPr(plhs[0]) = 1;
				}
				else
				{
					mexPrintf("Uncorrect call to this function.\n\n");
					helpOptions();
					plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
					*mxGetPr(plhs[0]) = 0;
				}
			}
		}
		if (nrhs>1)
		{
			option = string(buf);

			if ( option == "open" )
			{
				if ( !JI.Open(prhs[1]) ) // load a/c in JSBSim
					mexPrintf("JSBSim could not be started.\n");
				else
				{
					plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
					*mxGetPr(plhs[0]) = 1;
				}

			}
			if ( option == "get" )
			{
				
				// TO DO: if ( !JI.Get(prhs[1]) )
				double value;
				if ( !JI.GetPropertyValue(prhs[1],value) )
				{
					mexPrintf("Check property name.\n");
					// const char *buf = "PROPERTY NOT SET";
					// plhs[0]= mxCreateCharMatrixFromStrings((mwSize)1, (const char **)buf); 
					plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
					*mxGetPr(plhs[0]) = -9999;
					return;
				}
				else
					plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
					*mxGetPr(plhs[0]) = value;
			}
			if ( option == "set" )
			{
				if (nrhs>2)
				{
					if ( !JI.SetPropertyValue(prhs[1],prhs[2]) )
					{
						mexPrintf("Property could not be set.\n");
						plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
						*mxGetPr(plhs[0]) = 0;
					}
					else
					{
						plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
						*mxGetPr(plhs[0]) = 1;
					}
				}
				else
				{
					plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
					mexPrintf("ERROR: uncorrect use of 'set' option.\n");
					*mxGetPr(plhs[0]) = 0;
				}
			}
			if ( option == "SetVerbosity" )
			{
				if (nrhs==2)
				{
					if (!JI.SetVerbosity(prhs[1]) )
					{
						plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
						*mxGetPr(plhs[0]) = 0;
					}
					else
					{
						plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
						*mxGetPr(plhs[0]) = 1;
					}
				}
				else
				{
					plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
					mexPrintf("ERROR: uncorrect use of 'SetVerbosity' option.\n");
					*mxGetPr(plhs[0]) = 0;
				}
			}
			if ( option == "init" )
			{
				if ( !JI.Init(prhs[1]) )
				{
					mexPrintf("Initialization failed.\n");
					plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
					*mxGetPr(plhs[0]) = 0;
				}
				else
				{
					plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
					*mxGetPr(plhs[0]) = 1;
				}
			}
			if ( option == "dot" )
			{
				// create the vector that receives the calculated values
				const unsigned int nStates = 19;
				vector<double> statedot;
				for(unsigned int i=0;i<nStates;i++)
					statedot.push_back(0.0);  // init to zero

				if ( !JI.Init(prhs[1],statedot) ) // statedot modified by reference
				{
					mexPrintf("Calculation failed.\n");
					plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
					*mxGetPr(plhs[0]) = 0;
				}
				else // now statedot must be converted to a mxArray as needed
				{
					// see "doubleelement.c"
					//const mwSize dims[]={1,nStates};
					const mwSize dims[]={nStates};

					double data[nStates];
					for(unsigned int i=0;i<nStates;i++) // populate data
						data[i] = statedot[i];

					//for(unsigned int i=0;i<nStates;i++) mexPrintf("%d -> %f\n", i, statedot[i]);

					double *start_of_pr;
					size_t bytes_to_copy;

					(void) nlhs; // unused param
					// create a 1-by-nStates array of double
					//plhs[0] = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL);
					plhs[0] = mxCreateNumericArray(1,dims,mxDOUBLE_CLASS,mxREAL); // produces a Matlab column vector

					// populate the real part of the created array
					start_of_pr = (double *)mxGetData(plhs[0]);
					bytes_to_copy = nStates * mxGetElementSize(plhs[0]);
					memcpy(start_of_pr,data,bytes_to_copy);
				}
			}
		} // end of nrhs>1
	} // end of nrhs>0
	else
		helpOptions();

	return;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
