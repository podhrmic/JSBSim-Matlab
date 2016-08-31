/*
 * TestInterface.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: fwmav
 */

#include "TestInterface.h"
#include <models/FGAircraft.h>
#include <math/FGQuaternion.h>

//hack for lazy people
#define mexPrintf printf


TestInterface::TestInterface(FGFDMExec *fdmex)
{
	_ac_model_loaded = false;
	fdmExec = fdmex;
	propagate = fdmExec->GetPropagate();
	accel = fdmExec->GetAccelerations();
	//accel->InitModel();
	auxiliary = fdmExec->GetAuxiliary();
	aerodynamics = fdmExec->GetAerodynamics();
	propulsion = fdmExec->GetPropulsion();
	fcs = fdmExec->GetFCS();
	ic = new FGInitialCondition(fdmExec);
	verbosityLevel = TestInterface::eSilent;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TestInterface::~TestInterface(void)
{
	fdmExec = 0L;
	delete ic;
}
bool TestInterface::Open(string name)
{
	if (!fdmExec) return 0;
	string acName = name;
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

	if ( verbosityLevel == eVerbose )
		mexPrintf("\tLoading aircraft '%s' ...\n",acName.c_str());

    if ( ! fdmExec->LoadModel( rootDir + "aircraft",
                               rootDir + "engine",
                               rootDir + "systems",
                               acName))
	{
		if ( verbosityLevel == eVerbose )
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

	if ( verbosityLevel >= eVeryVerbose )
	{
		for (unsigned i=0; i<catalog.size(); i++)
			mexPrintf("%s\n",catalog[i].c_str());
	}
//***********************************************************************/

	return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool TestInterface::GetPropertyValue(const string prop, double& value)
{
	if (!fdmExec) return 0;
	if (!IsAircraftLoaded()) return 0;

	value = EasyGetValue(prop);

/*
	if ( !QueryJSBSimProperty(prop) )
	{
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tERROR: JSBSim could not find the property '%s' in the aircraft catalog.\n",prop.c_str());
		return 0;
	}
	mexPrintf("JSBSimInterface::GetPropertyValue: Asking for %s\n", prop.c_str());
	cout << "JSBSimInterface::GetPropertyValue: Asking for " << prop << endl;
	value = fdmExec->GetPropertyValue(prop);
	*/
	return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool TestInterface::SetPropertyValue(const string prop, const double value)
{
	if (!fdmExec) return 0;
	if (!IsAircraftLoaded()) return 0;

	if (!EasySetValue(prop,value)) // first check if an easy way of setting is implemented
	{
		if ( !QueryJSBSimProperty(prop) ) // then try to set the full-path property, e.g. '/fcs/elevator-cmd-norm'
		{
			if ( verbosityLevel == eVerbose )
				mexPrintf("\tERROR: JSBSim could not find the property '%s' in the aircraft catalog.\n",prop.c_str());
			return 0;
		}
		fdmExec->SetPropertyValue( prop, value );
	}
	return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool TestInterface::EasySetValue(const string prop, const double value)
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
		/*
		mexPrintf("\tEasy-set: alpha (deg) = %f,\n\tbeta (deg) = %f,\n\tgamma (deg) = %f\n",
			auxiliary->Getalpha()*180./M_PI,auxiliary->Getbeta()*180./M_PI,auxiliary->GetGamma()*180./M_PI);
		*/
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
		/*
		mexPrintf("\tEasy-set: alpha (deg) = %f,\n\tbeta (deg) = %f,\n\tgamma (deg) = %f\n",
			auxiliary->Getalpha()*180./M_PI,auxiliary->Getbeta()*180./M_PI,auxiliary->GetGamma()*180./M_PI);
		*/
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
		/*
		mexPrintf("\tEasy-set: alpha (deg) = %f,\n\tbeta (deg) = %f,\n\tgamma (deg) = %f\n",
			auxiliary->Getalpha()*180./M_PI,auxiliary->Getbeta()*180./M_PI,auxiliary->GetGamma()*180./M_PI);
		*/
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
double TestInterface::EasyGetValue(const string prop)
{
  if (prop == "set-running")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-Get: engine(s) running = %i\n",fdmExec->GetPropulsion()->GetEngine(0)->GetRunning());
    return fdmExec->GetPropulsion()->GetEngine(0)->GetRunning();
  }
  else if (prop == "u-fps")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: propagate->GetUVW(1);= %f\n", propagate->GetUVW(1));
    return propagate->GetUVW(1);;
  }
  else if (prop == "v-fps")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: propagate->GetUVW(2);= %f\n", propagate->GetUVW(2));
    return propagate->GetUVW(2);;
  }
  else if (prop == "w-fps")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: propagate->GetUVW(3);= %f\n", propagate->GetUVW(3));
    return propagate->GetUVW(3);;
  }
  else if (prop == "p-rad_sec")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: roll rate (rad/s) = %f\n",propagate->GetPQR(1));
    return propagate->GetPQR(1);;
  }
  else if (prop == "q-rad_sec")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: pitch rate (rad/s) = %f\n",propagate->GetPQR(2));
    return propagate->GetPQR(2);
  }
  else if (prop == "r-rad_sec")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: yaw rate (rad/s) = %f\n",propagate->GetPQR(3));
    return propagate->GetPQR(3);
  }
  else if (prop == "h-sl-ft")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: altitude over sea level (mt) = %f\n",propagate->GetAltitudeASLmeters());
    return propagate->GetAltitudeASLmeters();
  }
  else if (prop == "long-gc-deg")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: geocentric longitude (deg) = %f\n",propagate->GetLongitudeDeg());
    return propagate->GetLongitudeDeg();
  }
  else if (prop == "lat-gc-deg")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: geocentric latitude (deg) = %f\n",propagate->GetLatitudeDeg());
    return propagate->GetLatitudeDeg();
  }
  else if (prop == "phi-rad")
  {
    FGColumnVector3 euler = propagate->GetVState().GetEuler();
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: phi-rad = %f\n",euler.Entry(1));
    return euler.Entry(1);
  }
  else if (prop == "theta-rad")
  {
    FGColumnVector3 euler = propagate->GetVState().GetEuler();
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: theta-rad = %f\n",euler.Entry(2));
    return euler.Entry(2);
  }
  else if (prop == "psi-rad")
  {
    FGColumnVector3 euler = propagate->GetVState().GetEuler();
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: psi-rad = %f\n",euler.Entry(3));
    return euler.Entry(3);
  }
  else if (prop == "elevator-pos-rad")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: elevator pos (rad) = %f\n",fdmExec->GetFCS()->GetDePos());
    return fdmExec->GetFCS()->GetDePos();
  }
  else if (prop == "aileron-pos-rad")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-get: right aileron pos (rad) = %f\n",fdmExec->GetFCS()->GetDaRPos());
    return fdmExec->GetFCS()->GetDaRPos();
  }
  else if (prop == "rudder-pos-rad")
  {
    if ( verbosityLevel == eVeryVerbose )
      mexPrintf("\tEasy-set: rudder pos (deg) = %f\n",fdmExec->GetFCS()->GetDrPos());
    return GetFCS()->GetDrPos();
  }
  return 0;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool TestInterface::QueryJSBSimProperty(string prop)
{
	// mexPrintf("catalog size: %d\n",catalog.size());
	for (unsigned i=0; i<catalog.size(); i++)
	{
		//mexPrintf("__%s__\n",catalog[i].c_str());
		//if (catalog[i].find(prop) != std::string::npos) {
		//    std::cout << prop  << "found! Catalog entry is"  << catalog[i] << '\n';
		//}
		if (catalog[i]==prop) return 1;
		//if (catalog[i].find(prop) != std::string::npos) return 1;
	}
	return 0;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TestInterface::PrintCatalog()
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
bool TestInterface::Init()
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
	FGQuaternion Quatdot = propagate->GetQuaterniondot();
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
bool TestInterface::SetVerbosity(int ival)
{
	if (!fdmExec) return 0;
	// if (!IsAircraftLoaded()) return 0;
	// even when aircraft is null we must be able to set verbosity level


		if (ival <= eVeryVerbose)
		{
			SetVerbosity( (JIVerbosityLevel)ival );
		}
		else
			return 0;

	return 1;
}
