#include "JSBSimInterface.h"
#include <models/FGAircraft.h>
#include <models/FGAccelerations.h>
#include <math/FGQuaternion.h>

/*
 * Compiles with JSBSim checkout a59596f8f0d4c7b4f7ba24e99997bc2071d6cb72
 */

JSBSimInterface::JSBSimInterface(FGFDMExec *fdmex)
{
	_ac_model_loaded = false;
	fdmExec = fdmex;
	propagate = fdmExec->GetPropagate();
	accel = fdmExec->GetAccelerations();
	accel->InitModel();
	auxiliary = fdmExec->GetAuxiliary();
	aerodynamics = fdmExec->GetAerodynamics();
	propulsion = fdmExec->GetPropulsion();
	fcs = fdmExec->GetFCS();
	ic = new FGInitialCondition(fdmExec);
	verbosityLevel = JSBSimInterface::eSilent;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
JSBSimInterface::~JSBSimInterface(void)
{
	fdmExec = 0L;
	delete ic;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::Open(const mxArray *prhs)
{
	if (!fdmExec) return 0;

	char buf[128];
	mwSize buflen;
	buflen = mxGetNumberOfElements(prhs) + 1;
	mxGetString(prhs, buf, buflen);
	string acName = string(buf);
    string rootDir = "JSBSim/";

	//mexEvalString("plot(sin(0:.1:pi))");

	if ( fdmExec->GetAircraft()->GetAircraftName() != ""  )
	{
		if ( verbosityLevel >= eVerbose )
		{
			mexPrintf("\tERROR: another aircraft is already loaded ('%s').\n", fdmExec->GetAircraft()->GetAircraftName().c_str());
			mexPrintf("\t       To load a new aircraft, clear the mex file and start up again.\n");
		}
		return 0;
	}

	// JSBSim stuff

	if ( verbosityLevel >= eVerbose )
		mexPrintf("\tSetting up JSBSim with standard 'aircraft', 'engine', and 'system' paths.\n");

    fdmExec->SetAircraftPath (rootDir + "aircraft");
    fdmExec->SetEnginePath   (rootDir + "engine"  );
    fdmExec->SetSystemsPath  (rootDir + "systems" );

	if ( verbosityLevel >= eVerbose )
		mexPrintf("\tLoading aircraft '%s' ...\n",acName.c_str());

    if ( ! fdmExec->LoadModel( rootDir + "aircraft",
                               rootDir + "engine",
                               rootDir + "systems",
                               acName)) 
	{
		if ( verbosityLevel >= eVerbose )
			mexPrintf("\tERROR: JSBSim could not load the aircraft model.\n");
		return 0;
    }
	_ac_model_loaded = true;
	// Print AC name
	if ( verbosityLevel >= eVerbose )
		mexPrintf("\tModel %s loaded.\n", fdmExec->GetModelName().c_str() );

//***********************************************************************
	// populate aircraft catalog
	catalog = fdmExec->GetPropertyCatalog();

	if ( verbosityLevel == eVeryVerbose )
	{
		for (unsigned i=0; i<catalog.size(); i++) 
			mexPrintf("%s\n",catalog[i].c_str());
	}
//***********************************************************************/

	return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::GetPropertyValue(const mxArray *prhs1, double& value)
{
	if (!fdmExec) return 0;
	if (!IsAircraftLoaded()) return 0;

	char buf[128];
	mwSize buflen;
	buflen = mxGetNumberOfElements(prhs1) + 1;
	mxGetString(prhs1, buf, buflen);
	const string prop = string(buf);

	if (!EasyGetValue(prop, value)) // first check if an easy way of setting is implemented
	{
		if ( !QueryJSBSimProperty(prop) )
		{
			if ( verbosityLevel >= eVerbose )
				mexPrintf("\tERROR: JSBSim could not find the property '%s' in the aircraft catalog.\n",prop.c_str());
			return 0;
		}
		value = fdmExec->GetPropertyValue(prop);;
	}
	return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::SetPropertyValue(const mxArray *prhs1, const mxArray *prhs2)
{
	if (!fdmExec) return 0;
	if (!IsAircraftLoaded()) return 0;

	char buf[128];
	mwSize buflen;
	buflen = mxGetNumberOfElements(prhs1) + 1;
	mxGetString(prhs1, buf, buflen);
	const string prop = string(buf);
	double value = *mxGetPr(prhs2);

	if (!EasySetValue(prop,value)) // first check if an easy way of setting is implemented
	{
		if ( !QueryJSBSimProperty(prop) ) // then try to set the full-path property, e.g. '/fcs/elevator-cmd-norm'
		{
			if ( verbosityLevel >= eVerbose )
				mexPrintf("\tERROR: JSBSim could not find the property '%s' in the aircraft catalog.\n",prop.c_str());
			return 0;
		}
		fdmExec->SetPropertyValue( prop, value );
	}
	return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::SetPropertyValue(const string& prop, const double value)
{
		mxArray *p1 = mxCreateString(prop.c_str());
		mxArray *p2 = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(p2) = value;
		return SetPropertyValue(p1,p2);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::EasySetValue(const string& prop, const double value)
{
	if (prop == "set-running")
	{
		bool isrunning = false;
		if (value > 0) isrunning = true;
		for(unsigned i=0;i<fdmExec->GetPropulsion()->GetNumEngines();i++)
			fdmExec->GetPropulsion()->GetEngine(i)->SetRunning(isrunning);
		fdmExec->GetPropulsion()->GetSteadyState();
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: engine(s) running = %d\n",(int)isrunning);
		return 1;
	}
	else if (prop == "u-fps")
	{
		propagate->SetUVW(1,value);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: true flight speed (ft/s) = %f\n",auxiliary->GetVt());
		return 1;
	}
	else if (prop == "v-fps")
	{
		propagate->SetUVW(2,value);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: true flight speed (ft/s) = %f\n",auxiliary->GetVt());
		return 1;
	}
	else if (prop == "w-fps")
	{
		propagate->SetUVW(3,value);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: true flight speed (ft/s) = %f\n",auxiliary->GetVt());
		return 1;
	}
	else if (prop == "p-rad_sec")
	{
		propagate->SetPQR(1,value);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: roll rate (rad/s) = %f\n",propagate->GetPQR(1));
		return 1;
	}
	else if (prop == "q-rad_sec")
	{
		propagate->SetPQR(2,value);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: pitch rate (rad/s) = %f\n",propagate->GetPQR(2));
		return 1;
	}
	else if (prop == "r-rad_sec")
	{
		propagate->SetPQR(3,value);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: yaw rate (rad/s) = %f\n",propagate->GetPQR(3));
		return 1;
	}
	else if (prop == "h-sl-ft")
	{
		propagate->SetAltitudeASL(value);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: altitude over sea level (mt) = %f\n",propagate->GetAltitudeASLmeters());
		return 1;
	}
	else if (prop == "long-gc-deg")
	{
		propagate->SetLongitudeDeg(value);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: geocentric longitude (deg) = %f\n",propagate->GetLongitudeDeg());
		return 1;
	}
	else if (prop == "lat-gc-deg")
	{
		propagate->SetLatitudeDeg(value);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: geocentric latitude (deg) = %f\n",propagate->GetLatitudeDeg());
		return 1;
	}
	else if (prop == "phi-rad")
	{
		FGQuaternion Quat( value, propagate->GetEuler(2), propagate->GetEuler(3) );
		Quat.Normalize();
		FGPropagate::VehicleState vstate = propagate->GetVState();
		vstate.qAttitudeLocal = Quat;
		propagate->SetVState(vstate);
		propagate->Run(false); // vVel => gamma
		auxiliary->Run(false); // alpha, beta, gamma
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: phi -> quaternion = (%f,%f,%f,%f)\n",
				propagate->GetVState().qAttitudeLocal(1),propagate->GetVState().qAttitudeLocal(2),
				propagate->GetVState().qAttitudeLocal(3),propagate->GetVState().qAttitudeLocal(4));
		return 1;
	}
	else if (prop == "theta-rad")
	{
		FGQuaternion Quat( propagate->GetEuler(1), value, propagate->GetEuler(3) );
		Quat.Normalize();
		FGPropagate::VehicleState vstate = propagate->GetVState();
		vstate.qAttitudeLocal = Quat;
		propagate->SetVState(vstate);
		propagate->Run(false); // vVel => gamma
		auxiliary->Run(false); // alpha, beta, gamma
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: theta -> quaternion = (%f,%f,%f,%f)\n",
				propagate->GetVState().qAttitudeLocal(1),propagate->GetVState().qAttitudeLocal(2),
				propagate->GetVState().qAttitudeLocal(3),propagate->GetVState().qAttitudeLocal(4));
		return 1;
	}
	else if (prop == "psi-rad")
	{
		FGQuaternion Quat( propagate->GetEuler(1), propagate->GetEuler(2), value );
		Quat.Normalize();
		FGPropagate::VehicleState vstate = propagate->GetVState();
		vstate.qAttitudeLocal = Quat;
		propagate->SetVState(vstate);
		propagate->Run(false); // vVel => gamma
		auxiliary->Run(false); // alpha, beta, gamma
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: psi -> quaternion = (%f,%f,%f,%f)\n",
				propagate->GetVState().qAttitudeLocal(1),propagate->GetVState().qAttitudeLocal(2),
				propagate->GetVState().qAttitudeLocal(3),propagate->GetVState().qAttitudeLocal(4));
		return 1;
	}
	else if (prop == "elevator-cmd-norm")
	{
		fdmExec->GetFCS()->SetDeCmd(value);
		fdmExec->GetFCS()->Run(false);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: elevator pos (deg) = %f\n",fdmExec->GetFCS()->GetDePos()*180./M_PI);
		return 1;
	}
	else if (prop == "aileron-cmd-norm")
	{
		fdmExec->GetFCS()->SetDaCmd(value);
		fdmExec->GetFCS()->Run(false);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: right aileron pos (deg) = %f\n",fdmExec->GetFCS()->GetDaRPos()*180./M_PI);
		return 1;
	}
	else if (prop == "rudder-cmd-norm")
	{
		fdmExec->GetFCS()->SetDrCmd(value);
		fdmExec->GetFCS()->Run(false);
		propagate->Run(false);
		auxiliary->Run(false);
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\tEasy-set: rudder pos (deg) = %f\n",fdmExec->GetFCS()->GetDrPos()*180./M_PI);
		return 1;
	}
	return 0;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
double JSBSimInterface::EasyGetValue(const string& prop, double& value)
{
  if (prop == "set-running")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-Get: engine(s) running = %i\n",fdmExec->GetPropulsion()->GetEngine(0)->GetRunning());
    }
    value = (double)(fdmExec->GetPropulsion()->GetEngine(0)->GetRunning());
    return 1;
  }
  else if (prop == "u-fps")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: propagate->GetUVW(1);= %f\n", propagate->GetUVW(1));
    }
    value = propagate->GetUVW(1);
    return 1;
  }
  else if (prop == "v-fps")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: propagate->GetUVW(2);= %f\n", propagate->GetUVW(2));
    }
    value = propagate->GetUVW(2);
    return 1;
  }
  else if (prop == "w-fps")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: propagate->GetUVW(3);= %f\n", propagate->GetUVW(3));
    }
    value = propagate->GetUVW(3);
    return 1;
  }
  else if (prop == "p-rad_sec")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: roll rate (rad/s) = %f\n",propagate->GetPQR(1));
    }
    value = propagate->GetPQR(1);
    return 1;
  }
  else if (prop == "q-rad_sec")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: pitch rate (rad/s) = %f\n",propagate->GetPQR(2));
    }
    value = propagate->GetPQR(2);
    return 1;
  }
  else if (prop == "r-rad_sec")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: yaw rate (rad/s) = %f\n",propagate->GetPQR(3));
    }
    value = propagate->GetPQR(3);
    return 1;
  }
  else if (prop == "h-sl-ft")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: altitude over sea level (mt) = %f\n",propagate->GetAltitudeASLmeters());
    }
    value = propagate->GetAltitudeASLmeters();
    return 1;
  }
  else if (prop == "long-gc-deg")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: geocentric longitude (deg) = %f\n",propagate->GetLongitudeDeg());
    }
    value = propagate->GetLongitudeDeg();
    return 1;
  }
  else if (prop == "lat-gc-deg")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: geocentric latitude (deg) = %f\n",propagate->GetLatitudeDeg());
    }
    value = propagate->GetLatitudeDeg();
    return 1;
  }
  else if (prop == "phi-rad")
  {
    FGColumnVector3 euler = propagate->GetVState().qAttitudeLocal.GetEuler();
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: phi-rad = %f\n",euler.Entry(1));
    }
    value = euler.Entry(1);
    return 1;
  }
  else if (prop == "theta-rad")
  {
    FGColumnVector3 euler = propagate->GetVState().qAttitudeLocal.GetEuler();
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: theta-rad = %f\n",euler.Entry(2));
    }
    value = euler.Entry(2);
    return 1;
  }
  else if (prop == "psi-rad")
  {
    FGColumnVector3 euler = propagate->GetVState().qAttitudeLocal.GetEuler();
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: psi-rad = %f\n",euler.Entry(3));
    }
    value = euler.Entry(3);
    return 1;
  }
  else if (prop == "elevator-pos-rad")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: elevator pos (rad) = %f\n",fdmExec->GetFCS()->GetDePos());
    }
    value = fdmExec->GetFCS()->GetDePos();
    return 1;
  }
  else if (prop == "aileron-pos-rad")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-get: right aileron pos (rad) = %f\n",fdmExec->GetFCS()->GetDaRPos());
    }
    value = fdmExec->GetFCS()->GetDaRPos();
    return 1;
  }
  else if (prop == "rudder-pos-rad")
  {
    if ( verbosityLevel == eVeryVerbose )
    {
    	mexPrintf("\tEasy-set: rudder pos (deg) = %f\n",fdmExec->GetFCS()->GetDrPos());
    }
    value = fdmExec->GetFCS()->GetDrPos();
    return 1;
  }
  return 0;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::QueryJSBSimProperty(const string& prop)
{
	// mexPrintf("catalog size: %d\n",catalog.size());
	for (unsigned i=0; i<catalog.size(); i++)
	{
		//mexPrintf("__%s__\n",catalog[i].c_str());
		if (catalog[i].find(prop) != std::string::npos) return 1;
	}
	return 0;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void JSBSimInterface::PrintCatalog()
{
	if ( verbosityLevel >= eVerbose )
	{
		const std::string& name = fdmExec->GetAircraft()->GetAircraftName();
		mexPrintf("-- Property catalog for current aircraft ('%s'):\n", name.c_str());
		for (unsigned i=0; i<catalog.size(); i++)
			mexPrintf("%s\n",catalog[i].c_str());
		mexPrintf("-- end of catalog\n");
	}
	return;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::Init(const mxArray *prhs1)
{
	// Inspired by "refbook.c"
	// The argument prhs1 is pointer to a Matlab structure with two fields: name, value.
	// The Matlab user is forced to build such a structure first, then to pass it to the
	// mex-file. Example:
	//    >> ic(1).name = 'u'; ic(1).value = 80; (ft/s)
	//    >> ic(2).name = 'v'; ic(1).value =  0; (ft/s)
	//    >> ic(3).name = 'w'; ic(1).value =  0; (ft/s)
	//    >> ic(4).name = 'p'; ic(1).value =  0; (rad/s) % etc ...
	//    >> MexJSBSim('init', ic)

	//*************************************************
	// Set dt=0 first

	fdmExec->SuspendIntegration();
	
	//*************************************************

	bool success = 1;

	const char **fnames;       /* pointers to field names */
	const mwSize *dims;
	mxArray    *tmp;
	char       *pdata=NULL;
	int        ifield, nfields;
	mxClassID  *classIDflags;
	mwIndex    jstruct;
	mwSize     NStructElems;
	mwSize     ndim;

    // get input arguments
    nfields = mxGetNumberOfFields(prhs1);
    NStructElems = mxGetNumberOfElements(prhs1);
    // allocate memory  for storing classIDflags
    classIDflags = (mxClassID*)mxCalloc(nfields, sizeof(mxClassID));

    // check empty field, proper data type, and data type consistency;
	// and get classID for each field (see "refbook.c")
    for(ifield=0; ifield<nfields; ifield++) 
	{
		for(jstruct = 0; jstruct < NStructElems; jstruct++) 
		{
			tmp = mxGetFieldByNumber(prhs1, jstruct, ifield);
			if(tmp == NULL) 
			{
				if ( verbosityLevel == eVeryVerbose )
				{
					mexPrintf("%s%d\t%s%d\n", "FIELD: ", ifield+1, "STRUCT INDEX :", jstruct+1);
					mexErrMsgTxt("Above field is empty!");
				}
				return 0;
			} 
			if(jstruct==0) 
			{
				if( (!mxIsChar(tmp) && !mxIsNumeric(tmp)) || mxIsSparse(tmp)) 
				{
					if ( verbosityLevel == eVeryVerbose )
					{
						mexPrintf("%s%d\t%s%d\n", "FIELD: ", ifield+1, "STRUCT INDEX :", jstruct+1);
						mexErrMsgTxt("Above field must have either string or numeric non-sparse data.");
					}
					return 0;
				}
				classIDflags[ifield]=mxGetClassID(tmp); 
			} 
			else 
			{
				if (mxGetClassID(tmp) != classIDflags[ifield]) 
				{
					if ( verbosityLevel == eVeryVerbose )
					{
						mexPrintf("%s%d\t%s%d\n", "FIELD: ", ifield+1, "STRUCT INDEX :", jstruct+1);
						mexErrMsgTxt("Inconsistent data type in above field!"); 
					}
					return 0;
				} 
				else if(!mxIsChar(tmp) && 
					  ((mxIsComplex(tmp) || mxGetNumberOfElements(tmp)!=1)))
				{
					if ( verbosityLevel == eVeryVerbose )
					{
						mexPrintf("%s%d\t%s%d\n", "FIELD: ", ifield+1, "STRUCT INDEX :", jstruct+1);
						mexErrMsgTxt("Numeric data in above field must be scalar and noncomplex!"); 
					}
					return 0;
				}
			}
		}
    }
    /* allocate memory  for storing pointers */
    fnames = (const char **)mxCalloc(nfields, sizeof(*fnames));
    /* get field name pointers */
    for (ifield=0; ifield< nfields; ifield++)
	{
		fnames[ifield] = mxGetFieldNameByNumber(prhs1,ifield);
    }
	// At this point we have extracted from prhs1 the vector of 
	// field names fnames of nfields elements.
	// nfields is the number of fields in the passed Matlab struct (ic).
	// It may have more fields, but the first two must be "name" and "value"
	// The structure possesses generally a number of NStructElems elements.

    ndim = mxGetNumberOfDimensions(prhs1);
    dims = mxGetDimensions(prhs1);

	// loop on the element of the structure
	for (jstruct=0; jstruct<NStructElems; jstruct++) 
	{
		string prop = "";
		double value = -99.;

		// scan the fields
		// the first two must be "name" and "value"
		for(ifield=0; ifield<2; ifield++) // for(ifield=0; ifield<nfields; ifield++) // nfields=>2
		{
			tmp = mxGetFieldByNumber(prhs1,jstruct,ifield);
			if( mxIsChar(tmp) ) //  && (fnames[ifield]=="name") the "name" field
			{
				// mxSetCell(fout, jstruct, mxDuplicateArray(tmp));
				char buf[128];
				mwSize buflen;
				buflen = mxGetNumberOfElements(tmp) + 1;
				mxGetString(tmp, buf, buflen);
				prop = string(buf);
				//mexPrintf("field name: %s\n",prop.c_str());
			}
			else  // the "value" field
			{
				value = *mxGetPr(tmp);
				//mexPrintf("field value %f\n",value);
			}
		}
		//----------------------------------------------------
		// now we have a string in prop and a double in value
		// we got to set the property value accordingly
		//----------------------------------------------------
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("Property name: '%s'; to be set to value: %f\n",prop.c_str(),value);

		//----------------------------------------------------
		// Note: the time step is set to zero at this point, so that all calls 
		//       to propagate->Run() will not advance the vehicle state in time
		//----------------------------------------------------
		// Now pass prop and value to the member function
		success = success && SetPropertyValue(prop,value); // EasySet called here

		//mexPrintf("success '%d'; \n",(int)success);
    }
	// free memory
    mxFree(classIDflags);
	mxFree((void *)fnames);

	//---------------------------------------------------------------
	// see "FGInitialConditions.h"
	// NOTE:

	double vt = 
		sqrt(
			fdmExec->GetPropagate()->GetUVW(1) * fdmExec->GetPropagate()->GetUVW(1) + // to do State ??
			fdmExec->GetPropagate()->GetUVW(2) * fdmExec->GetPropagate()->GetUVW(2) +
			fdmExec->GetPropagate()->GetUVW(3) * fdmExec->GetPropagate()->GetUVW(3) );


	//mexPrintf("Vt = %f\n",fdmExec->GetAuxiliary()->GetVt());


	fdmExec->Run();
	fdmExec->ResumeIntegration();


	// Calculate state derivatives
	//fdmExec->GetAccelerations()->CalculatePQRdot();      // Angular rate derivative (should be done automatically)
	//fdmExec->GetAccelerations()->CalculateUVWdot();      // Translational rate derivative (should be done automatically)
	//fdmExec->GetPropagate()->CalculateQuatdot();     // Angular orientation derivative (should be done automatically)
	//fdmExec->GetPropagate()->CalculateLocationdot(); // Translational position derivative (should be done automatically)

	// FDMExec.GetAuxiliary()->Run();
	FGColumnVector3 euler_rates = auxiliary->GetEulerRates();
	FGColumnVector3 position_rates = propagate->GetUVW();

	/*
	 * dunno what is wrong here
	_udot = accel->GetUVWdot(1);
	_vdot = fdmExec->GetAccelerations()->GetUVWdot(2);
	_wdot = fdmExec->GetAccelerations()->GetUVWdot(3);
	_pdot = fdmExec->GetAccelerations()->GetPQRdot(1);
	_qdot = fdmExec->GetAccelerations()->GetPQRdot(2);
	_rdot = fdmExec->GetAccelerations()->GetPQRdot(3);
	*/
	FGQuaternion Quatdot = accel->GetQuaterniondot();
	_q1dot = Quatdot(1);
	_q2dot = Quatdot(2);
	_q3dot = Quatdot(3);
	_q4dot = Quatdot(4);
	_phidot   = euler_rates(1);
	_thetadot = euler_rates(2);
	_psidot   = euler_rates(3);
	_xdot = position_rates(1);
	_ydot = position_rates(2);
	_zdot = position_rates(3);
	_hdot = propagate->Gethdot();
	_alphadot = auxiliary->Getadot();
	_betadot = auxiliary->Getbdot();

	if ( verbosityLevel >= eVerbose )
	{
		mexPrintf("\tState derivatives calculated.\n");

		mexPrintf("\tV true %f (ft/s)\n",auxiliary->GetVt());
		//mexPrintf("\t[u_dot, v_dot, w_dot] = [%f, %f, %f] (ft/s/s)\n",
		//	propagate->GetUVWdot(1),propagate->GetUVWdot(2),propagate->GetUVWdot(3));
		//mexPrintf("\t[p_dot, q_dot, r_dot] = [%f, %f, %f] (rad/s/s)\n",
		//	propagate->GetPQRdot(1),propagate->GetPQRdot(2),propagate->GetPQRdot(3));
		mexPrintf("\t[x_dot,y_dot,z_dot] = [%f, %f, %f] (ft/s)\n",
			position_rates(1),position_rates(2),position_rates(3));
		mexPrintf("\t[phi_dot,theta_dot,psi_dot] = [%f, %f, %f] (rad/s)\n",
			euler_rates(1),euler_rates(2),euler_rates(3));
		mexPrintf("\t[h_dot, alpha_dot, beta_dot] = [%f (ft/s/s),%f (deg/sec),%f (deg/sec)]\n",
			_hdot,_alphadot*180/M_PI,_betadot*180/M_PI);
	}

	if (!success)
	{
		if ( verbosityLevel >= eVerbose )
			mexPrintf("\tERROR: One or more or all the required properties could not be initialized.\n");
		return 0;
	}
	else
		return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::Init(const mxArray *prhs1, vector<double>& statedot)
{
	if (statedot.size() != 19) return 0;
	if (!Init(prhs1))
	{
		if ( verbosityLevel >= eVerbose )
			mexPrintf("\tERROR: could not calculate dotted states correctly.\n");
		return 0;
	}
	statedot[ 0] = _udot;
	statedot[ 1] = _vdot;
	statedot[ 2] = _wdot;
	statedot[ 3] = _pdot;
	statedot[ 4] = _qdot;
	statedot[ 5] = _rdot;
	statedot[ 6] = _q1dot;
	statedot[ 7] = _q2dot;
	statedot[ 8] = _q3dot;
	statedot[ 9] = _q4dot;
	statedot[10] = _xdot;
	statedot[11] = _ydot;
	statedot[12] = _zdot;
	statedot[13] = _phidot;
	statedot[14] = _thetadot;
	statedot[15] = _psidot;
	statedot[16] = _hdot;
	statedot[17] = _alphadot;
	statedot[18] = _betadot;

	return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::SetVerbosity(const mxArray *prhs1)
{
	if (!fdmExec) return 0;

	if ( mxIsChar(prhs1) )
	{
		char buf[128];
		mwSize buflen;
		buflen = mxGetNumberOfElements(prhs1) + 1;
		mxGetString(prhs1, buf, buflen);
		string prop = "";
		prop = string(buf);

		if ( (prop == "silent") ||
			 (prop == "Silent") ||
			 (prop == "SILENT")
			)
		{
			SetVerbosity( (JIVerbosityLevel)0 );
		}
		else if ( (prop == "verbose") ||
			      (prop == "Verbose") ||
			      (prop == "VERBOSE")
			)
		{
			SetVerbosity( (JIVerbosityLevel)1 );
		}
		else if ( (prop == "very verbose") ||
			      (prop == "Very Verbose") ||
			      (prop == "VERY VERBOSE")
			)
		{
			SetVerbosity( (JIVerbosityLevel)2 );
		}
		else
			return 0;
	}
	else if ( mxIsNumeric(prhs1) )
	{
		double value = 0;
		value = *mxGetPr(prhs1);
		int ival = (int) value;
		if (ival <= eVeryVerbose)
		{
			SetVerbosity( (JIVerbosityLevel)ival );
		}
		else
			return 0;
	}

	return 1;
}

