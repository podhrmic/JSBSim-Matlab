/*
 * test.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: fwmav
 */

#include <iostream>
#include <string>
#include <vector>
#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <models/FGFCS.h>

#include "TestInterface.h"

using namespace std;

// function prototypes
// ...

// this object should be persistent in memory until
// the MEX-function is cleared by Matlab


int main(int argc, char **argv)
{
	JSBSim::FGFDMExec FDMExec;
	TestInterface JI(&FDMExec);

	string aircraftName = "c172r";
	string option = "-1";

	if ( !JI.Open(aircraftName) ) // load a/c in JSBSim
		printf("JSBSim could not be started.\n");
	else
	{
		printf("JSBSim started.\n");
	}

	if (JI.Init())
	{
		printf("Init done!\n");
	}
	else
	{
		printf("Init failed!\n");
	}

	//JI.PrintCatalog();

	FDMExec.SetRootDir("/home/fwmav/Paparazzi/Code/JSBSim-Matlab");
	cout << "Root dir: " << FDMExec.GetRootDir() << endl;

	if (JI.SetVerbosity(2)){
		printf("Verbosity set!\n");
	}
	else {
		printf("Verbosity NOT set!\n");
	}

	double value = -1;
	string prop = "fcs/rudder-cmd-norm";
	printf("Setting %s.\n",prop.c_str());

	/*
	FDMExec.SetPropertyValue(prop,value);
	printf("Setting done.\n");

	printf("Getting %s\n", prop.c_str());
	double value2 = FDMExec.GetPropertyValue(prop);
	printf("Done. Set %s = %f, got %s = %f \n", prop.c_str(), value, prop.c_str(), value2);
	*/

	if (!JI.SetPropertyValue(prop,value)){
		printf("Check property name.\n");
	}
	else {
		printf("Success.\n");
	}

	JI.RunFDMExec();

	printf("Getting %s.\n",prop.c_str());
	if ( !JI.GetPropertyValue(prop,value) )
	{
		printf("Check property name.\n");
	}
	else
		printf("Value = %f.\n", value);
}

