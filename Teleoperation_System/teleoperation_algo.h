
// Omega To Phantom Teleoperation System Algorithm Header

// Phantom lib
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#include <cstdio>
#include <cassert>

#if defined(WIN32)
# include <conio.h>
#else
# include "conio.h"
#endif

#pragma warning (disable:4996)

//#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
//#include <HDU/hduMatrix.h>


//#include <iostream>
#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>
#include <windows.h>
#include <vector>

// loop rate detect
//#include <time.h>
#include <ctime>

// POPC lib  <- koreatech ver 1.0
#include "KTBioLabLib.hpp"
#include "teleoperation_data_structure.h"

// Omega lib 
#include "dhdc.h"
#include "drdc.h"

// Omega To Phantom Dimension Change Class
#include "OmegaToPhantomDimension.h"

// Shared Memory lib
#include <Windows.h>
#include <tchar.h>
#include <conio.h>


// Data Saving Parameter
#define counter2Time 1000
#define T 0.001
int n = counter2Time / T;


#define T 0.001  // for discrete system

// Delay Data Saving
std::vector<mst2net> omega_data_stack;
std::vector<slv2net> phantom_data_stack;

// Shared memory - FT sensor data
FTvalue* ft_readbuf;
FTSENSOR* ft_sensor;


// POPC Library class 
KTBioLabLib::AlpopcLib *m_PopcClass;

// Rate Mode Libraru
KTBioLabLib::AlteleoperationLib *m_RatemodeClass;

// Calbration state
bool Calibration_state = false;
hduVector3Dd init_position(0,0,0);
hduVector3Dd position(0,0,0);

// Slave Phantom Gain (Kp, Kd)
double K_slave[3] = {0.2, 0.2, 0.2};
double slv_damping[3] = {0.16, 0.16, 0.16};

double *moving_aver_x;
double *moving_aver_y;
double *moving_aver_z;

int moving_aver_num = 25;

// Shared memory handler
HANDLE hMapping;


// SSI test
SSIstructure SSI_data[3];

void MovingEverageFilter(double& moving_counter, bool & moving_counter_checker, double value, double& output_value){

}


HDCallbackCode HDCALLBACK DeviceCalibrate(void *pUserData)
{
	hdBeginFrame(hdGetCurrentDevice());
		hdGetDoublev(HD_CURRENT_POSITION,init_position);
	hdEndFrame(hdGetCurrentDevice());
	return HD_CALLBACK_DONE;
}

int bi_teleoperation_data(mst2net *net_omega, mst2net *net_omega_delay, slv2net *net_phantom, slv2net *net_phantom_delay, tel_state *state_)
{
	// master omega generate delay
	static mst2net m_prev_delayed_data; 
	static bool m_first_time_checker = false;

	if(!state_->TimeDelay_Check){
		memcpy(net_omega_delay,net_omega,sizeof(mst2net));
	}else{
		omega_data_stack.push_back(*net_omega);

		// Artificial Time Delay
		double time_diff = (double(clock()) - omega_data_stack.front().time_stamp);
		printf("master time_diff : %f\n",time_diff);
		if(time_diff >= state_->Delay_time){

		  //delayed_data = SlaveVelocityData.front();
		  memcpy(net_omega_delay,&omega_data_stack.front(),sizeof(mst2net));

		  if(m_first_time_checker == false){
		  	memcpy(&m_prev_delayed_data,net_omega_delay,sizeof(mst2net));
		  	m_first_time_checker = true;
		  }

		  // Artificial Packet Loss
		  srand((unsigned int)time_diff);
		  if(state_->Packet_loss > 0){
		    int r = rand()%100; //0~99 random integer number
		    if(r < state_->Packet_loss){
		      memcpy(net_omega_delay,&m_prev_delayed_data,sizeof(mst2net));
		    }
		  }
		  
		  omega_data_stack.erase(omega_data_stack.begin());
		  memcpy(&m_prev_delayed_data,&net_omega_delay,sizeof(mst2net));
		}
	}
	
	double slv_ForceTorque[3];
	double slv_energy[3];


	// slave phantom generate delay
	static slv2net s_prev_delayed_data; 
	static bool s_first_time_checker = false;

	if(!state_->TimeDelay_Check){
		memcpy(net_phantom_delay,net_phantom,sizeof(slv2net));
	}else{
		phantom_data_stack.push_back(*net_phantom);

		// Artificial Time Delay
		double time_diff = (double(clock()) - phantom_data_stack.front().time_stamp);
		printf("slave time_diff : %f\n",time_diff);
		if(time_diff >= state_->Delay_time){

		  //delayed_data = SlaveVelocityData.front();
		  memcpy(net_phantom_delay,&phantom_data_stack.front(),sizeof(slv2net));

		  if(s_first_time_checker == false){
		  	memcpy(&s_prev_delayed_data,net_phantom_delay,sizeof(slv2net));
		  	s_first_time_checker = true;
		  }

		  // Artificial Packet Loss
		  srand((unsigned int)time_diff);
		  if(state_->Packet_loss > 0){
		    int r = rand()%100; //0~99 random integer number
			if(r < state_->Packet_loss){
		      memcpy(net_phantom_delay,&s_prev_delayed_data,sizeof(slv2net));
		    }
		  }
		  
		  phantom_data_stack.erase(phantom_data_stack.begin());
		  memcpy(&s_prev_delayed_data,net_phantom_delay,sizeof(slv2net));
		}
	}
	
	return 1;
}

int ForceFeedback(slv2net *phantom_, m_omega *omega_)
{

	static double Dt = 0.0;
	static clock_t old_t = clock();
	Dt = double(clock() - old_t) / 1000.0;
	old_t = clock();
	//if(!(Dt>0)) return -1;

	//Dt = 1.0/frequency;
	//Dt = 0.001;

	double Fs[3] = {phantom_->SlvForceTorque[0], phantom_->SlvForceTorque[1], phantom_->SlvForceTorque[2]}; // Force from Slave
	static double Xm[3] =  {0.0, 0.0, 0.0}; // Position of Master Device
	double Vm[3] = {omega_->m_vel[0], omega_->m_vel[1], omega_->m_vel[2]}; // Velocity of Master Device
	double final_forcefeedback[3] = {0.0, 0.0, 0.0};

	if(omega_->m_indexing){
		// Master Position Update
		for(unsigned i=0; i<3; i++){
			Xm[i] += Vm[i]*Dt;
		}

		// Force Saturation
		for(unsigned int i=0;i<3;i++){
			if(Fs[i] > MAX_FEEDBACK_FORCE){
			  Fs[i] = MAX_FEEDBACK_FORCE;
			}
			else if(Fs[i] < -MAX_FEEDBACK_FORCE){
			  Fs[i] = -MAX_FEEDBACK_FORCE;
			}
			omega_->m_forcefeedback[i] = Fs[i];
		}
	}
	return 1;
}

int master_initialize(tel_state *state_)
{
	// master omega initialize
	// message
	int major, minor, release, revision;
	dhdGetSDKVersion (&major, &minor, &release, &revision);
	printf ("\n");
	printf ("Force Dimension - Omega %d.%d.%d.%d\n", major, minor, release, revision);
	printf ("(C) 2014 Force Dimension\n");
	printf ("All Rights Reserved.\n\n");

	// required to change asynchronous operation mode
	dhdEnableExpertMode ();

	// open the first available device
	//if (dhdOpenSerial(SERIAL_NUMBER_OMEGA3) < 0) {
	if (drdOpen() < 0) {	
		printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
	  	dhdSleep (2.0);
	  	// system state 
		//state_->system_state = SYSTEM_OFFLINE;
	    return -1;
	}

	// print out device identifier
	if (!drdIsSupported ()) {
		printf ("unsupported device\n");
		printf ("exiting...\n");
		dhdSleep (2.0);
		drdClose ();
		return -1;
	}

	// serial Number 
	//ushort Serial_;
    //dhdGetSerialNumber(&Serial_);
    //std::cout << Serial_ << std::endl;

	// identify device
    printf ("%s device detected\n\n", dhdGetSystemName());

	// perform auto-initialization
	if (!drdIsInitialized () && drdAutoInit () < 0) {
		printf ("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr ());
		dhdSleep (2.0);
		return -1;
	}
	
	// start robot control loop
	if (drdStart () < 0) {
		printf ("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr ());
		dhdSleep (2.0);
		drdClose ();
		return -1;
	}

	// center of workspace
	double nullPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.0,  // base  (translations)
									0.0, 0.0, 0.0,  // wrist (rotations)
									0.0 };          // gripper

	 // move to center
	drdMoveTo (nullPose);

	// stop regulation thread (but leaves forces on)
	drdStop (true);

	// display instructions
	printf ("press BUTTON or 'q' to quit\n\n");

	// enable force
	dhdEnableForce (DHD_ON);

	// system state 
	//state_->system_state = SYSTEM_ONLINE;

	return 1;
}
int master_popc(slv2net *net_phantom, m_omega *omega_, mst_PO *PO, tel_state *state_)
{
	
	// Force from Slave
    static double Fs[3] = { net_phantom->SlvForceTorque[0], net_phantom->SlvForceTorque[1], net_phantom->SlvForceTorque[2] };
    // Position of Master Device
    static double Xm[3] = { 0.0, 0.0, 0.0 };
    // velcoity of Master Device
    static double Vm[3] = { omega_->m_vel[0], omega_->m_vel[1], omega_->m_vel[2] };

    // Postion of virtual mass
    static double X_vms[3] = { 0.0, 0.0, 0.0 };
    // Velocity of virtual mass
    static double V_vms[3] = { 0.0, 0.0, 0.0 };

    // Force for virtual mass spring damper
    static double Fs_vms[3] = { net_phantom->SlvForceTorque[0], net_phantom->SlvForceTorque[1], net_phantom->SlvForceTorque[2] };
    static double final_forceFeedback[3] = { 0.0, 0.0, 0.0 };

    // Passivity Observer variable
    static double E_master_in[3] = { 0.0, 0.0, 0.0 };
    static double E_master_out[3] = { 0.0, 0.0, 0.0 };
    static double E_master_out_origin[3] = { 0.0, 0.0, 0.0 };
    static double Fs_origin[3] = { net_phantom->SlvForceTorque[0], net_phantom->SlvForceTorque[1], net_phantom->SlvForceTorque[2] };
    static double E_slave_in[3] = { net_phantom->SlvEnergy[0], net_phantom->SlvEnergy[1], net_phantom->SlvEnergy[2] };

	static double E_master_remain[3] = {0.0, 0.0, 0.0};
    // Passivity Controller Parameter edit (necessariness)
	m_PopcClass->POPC_Intialize(0.0, 0.0, state_->v_mass, state_->v_spring, state_->v_damping);
	
	for(int i=0;i<DOF;i++)
	{
		// Force from Slave
		Fs[i] = net_phantom->SlvForceTorque[i];
		// velcoity of Master Device
		Vm[i] = omega_->m_vel[i];
		// Force for virtual mass spring damper
		Fs_vms[i] = Fs[i];  
		// Original Force
		Fs_origin[i] = Fs[i];
		// Slave Energy
		E_slave_in[i] = net_phantom->SlvEnergy[i];

		// Virtual Mass Spring Damper adaptive
		Xm[i] += Vm[i] * omega_->m_dt;
		E_slave_in[i] = net_phantom->SlvEnergy[i];

		if(state_->POPC_Act){

			if(state_->virtual_mass_spring_on){
				// Vritual Mass Spring Version
				//m_PopcClass->ImpedancePOPC_modified(E_slave_in[i], E_master_in[i], E_master_out_origin[i], E_master_out[i], E_master_remain[i], Fs[i], 
				//	Fs_vms[i], Xm[i], X_vms[i], Vm[i], V_vms[i], omega_->m_dt);
				//FOUR_CHANEL_COMMUNICATION
				switch(state_->Control_mathod)
				{
					case NORMAL_POPC :
					{
						m_PopcClass->ImpedancePOPC(E_slave_in[i], E_master_in[i], E_master_out_origin[i], E_master_out[i], Fs[i],
							Fs_vms[i], Xm[i], X_vms[i], Vm[i], V_vms[i], omega_->m_dt);
						break;
					}
					case POSITION_DRIFT_POPC :
					{
						m_PopcClass->ImpedancePOPC_PDC(E_slave_in[i], E_master_in[i], E_master_out_origin[i], E_master_out[i], E_master_remain[i], Fs[i], 
							Fs_vms[i], Xm[i], X_vms[i], Vm[i], V_vms[i], omega_->m_dt);
						break;
					}
					case POSITION_DRIFT_ADVANCE_POPC :
					{
						m_PopcClass->ImpedancePOPC_PDC(E_slave_in[i], E_master_in[i], E_master_out_origin[i], E_master_out[i], E_master_remain[i], Fs[i], 
							Fs_vms[i], Xm[i], X_vms[i], Vm[i], V_vms[i], omega_->m_dt);
						break;
					}
					case FOUR_CHANEL_COMMUNICATION :
					{

					}
					default :
					{
						break;
					}
				}
				
				// Master Feedback Force input
				omega_->m_forcefeedback[i] = Fs_vms[i];
				// Master Energy input
				omega_->m_energy[i] = E_master_in[i];

			}else{
				// Non-Virtual Mass Spring Version
				// Now Not Activate Input --- popc.NONImpedancePOPC()
				m_PopcClass->ImpedancePOPC_NonVMS(E_slave_in[i], E_master_in[i], E_master_out_origin[i], E_master_out[i], Fs[i],
					Fs_vms[i], Vm[i], omega_->m_dt);
				// Master Feedback Force input
				omega_->m_forcefeedback[i] = Fs_vms[i];
				// Master Energy input
				omega_->m_energy[i] = E_master_in[i];
			}
		}
		else{

			E_slave_in[i] = 0.0;
			E_master_out[i] = 0.0;
			E_master_in[i] = 0.0;
			E_master_out_origin[i] = 0.0;

			omega_->m_forcefeedback[i] = Fs[i];
		}

		// Master Passivity Observer & Passivity Controller Data logging
		PO->m_Data_Em_in[i] = E_master_in[i];
		PO->m_Data_Em_out_origin[i] = E_master_out_origin[i];
		PO->m_Data_Em_out[i] = E_master_out[i];
		PO->m_Data_Es_in[i] = E_slave_in[i];
		PO->m_Data_Epc[i] = E_master_remain[i];
		PO->m_Data_Fs_origin[i] = Fs_origin[i];
		PO->m_Data_Fs[i] = Fs[i];
		PO->m_Data_Fvms[i] = Fs_vms[i];
		PO->m_Data_Vm[i] = Vm[i];
	}
	PO->m_Dt = omega_->m_dt;

	//printf("E_slave in : %f,   E master out : %f,    Force : %f,   Velocity : %f\n",E_slave_in[1],PO->m_Data_Em_out[1],Fs[1],Vm[1]);


	return 1;
}

int master_omega_gen(slv2net *net_phantom, mst2net *net_omega, m_omega *omega_, mst_PO *m_PO, tel_state *state_,bool &loop_state)
{

	// Determine sampling time
	static double Dt = 0.0;
	static clock_t old_t = clock();
	Dt = double(clock() - old_t) / 1000.0;
	old_t = clock();
	
	//if(!(Dt>0)) return -1;
	

	// sampling time by omega
	Dt = 0.001;
	omega_->m_dt = Dt;
	
	//---Get Omega State
	double pose[3]={0.0,};
	double angular_vel[3]={0.0,};
	dhdGetPosition(&pose[0], &pose[1], &pose[2]);
	dhdGetAngularVelocityRad(&angular_vel[0],&angular_vel[1],&angular_vel[2]);
	//dhdGetOrientationRad(&mst->m_orientation[0], &mst->m_orientation[1], &mst->m_orientation[2]);
	double vel[3]={0.0,};
	dhdGetLinearVelocity (&vel[0], &vel[1], &vel[2]);	
	
	
	

	OmegaToPhantomDimension omega2phantom;
	
	double dimension_changed_vel[3]={0.0,}; 
	double dimension_changed_pose[3]={0.0,};
	
	
	omega2phantom.OmegaToPhantomDimensionChange(vel[0],vel[1],vel[2],dimension_changed_vel[0],dimension_changed_vel[1],dimension_changed_vel[2]);
	omega2phantom.OmegaToPhantomDimensionChange(pose[0],pose[1],pose[2],dimension_changed_pose[0],dimension_changed_pose[1],dimension_changed_pose[2]);
	
	double omega_vel[3] ={0.0,};
	double omega_m_pose[3] ={0.0,};
	for(int i=0;i<3;i++){
		omega_vel[i] = dimension_changed_vel[i] * 1000.0;
		omega_m_pose[i] = dimension_changed_pose[i] * 1000.0;
	}
	
	// time stamp by omega
	net_omega->time_stamp = double(clock());

	static int moving_counter[3] = {0,0,0};
	static bool moving_counter_checker[3] = {false,};

	// indexing system
	if(dhdGetButtonMask()){
	//if(true){
		net_omega->m_indexing = true;
		omega_->m_indexing = true;

		

		for(int i = 0;i<DOF;i++){
			omega_->m_pose[i] = omega_m_pose[i];
			net_omega->MstVel[i] = omega_vel[i];
		}
	}else{
		net_omega->m_indexing = false;
		omega_->m_indexing = false;
		for(int i =0;i<DOF;i++){
			omega_->m_vel[i] = 0.0;
			omega_->m_pose[i] = 0.0;
			omega_->m_energy[i] = 0.0;
			omega_vel[i] = 0.0;
			
			net_omega->MstVel[i] = omega_->m_vel[i];	
		}
	}

	double output_value[3] = {0.0,0.0,0.0};
	// moving average filter
	if(moving_counter[0] != moving_aver_num-2){
		moving_aver_x[moving_counter[0]] = double(omega_vel[0]);
		moving_counter[0] = moving_counter[0] + 1;
	}else{
		moving_aver_x[moving_counter[0]] = double(omega_vel[0]);
		moving_counter_checker[0] = true;
		moving_counter[0] = 0;
	}
	double value_tmp_x = 0;
	if(moving_counter_checker[0] == true){
		for(int i=0;i<moving_aver_num;i++){
			value_tmp_x += moving_aver_x[i];
		}
		output_value[0] = value_tmp_x / double(moving_aver_num);
		omega_->m_vel[0] = output_value[0];
	}

	// moving average filter
	if(moving_counter[1] != moving_aver_num-2){
		moving_aver_y[moving_counter[1]] = double(omega_vel[1]);
		moving_counter[1] = moving_counter[1] + 1;
	}else{
		moving_aver_y[moving_counter[1]] = double(omega_vel[1]);
		moving_counter_checker[1] = true;
		moving_counter[1] = 0;
	}
	double value_tmp_y = 0;
	if(moving_counter_checker[1] == true){
		for(int i=0;i<moving_aver_num;i++){
			value_tmp_y += moving_aver_y[i];
		}
		output_value[1] = value_tmp_y / double(moving_aver_num);
		omega_->m_vel[1] = output_value[1];
	}


	// moving average filter
	if(moving_counter[2] != moving_aver_num-2){
		moving_aver_z[moving_counter[2]] = double(omega_vel[2]);
		moving_counter[2] = moving_counter[2] + 1;
	}else{
		moving_aver_z[moving_counter[2]] = double(omega_vel[2]);
		moving_counter_checker[2] = true;
		moving_counter[2] = 0;
	}
	double value_tmp_z = 0;
	if(moving_counter_checker[2] == true){
		for(int i=0;i<moving_aver_num;i++){
			value_tmp_z += moving_aver_z[i];
		}
		output_value[2] = value_tmp_z / double(moving_aver_num);
		omega_->m_vel[2] = output_value[2];
	}

	// master Time Domain Passivity Approach Active(PO/PC)
	master_popc(net_phantom, omega_, m_PO, state_);
	if(!omega_->m_indexing){
		for(int i =0;i<DOF;i++)
			omega_->m_forcefeedback[i] = 0.0;
	}
	
	static double prev_err[3] = {0.0,0.0,0.0};
	static double err[3] = {0.0,0.0,0.0};

	for(int i=0;i<DOF;i++){
		prev_err[i] = err[i];
		//printf("dimension_changed_pose : %f\n",dimension_changed_pose[i]);
		err[i] = 0.0 - dimension_changed_pose[i];
		//m_RatemodeClass->RateModeControl(omega_->m_indexing, err[i], err[i] - prev_err[i], omega_m_pose[i], 
		//	net_omega->MstVel[i],omega_->m_forcefeedback[i]);
		
		// Energy Input
		net_omega->MstEnergy[i] = omega_->m_energy[i];

		// Force Saturation
		if(omega_->m_forcefeedback[i]  > MAX_FEEDBACK_FORCE){
		  omega_->m_forcefeedback[i]  = MAX_FEEDBACK_FORCE;
		}else if(omega_->m_forcefeedback[i]  < -MAX_FEEDBACK_FORCE){
		  omega_->m_forcefeedback[i]  = -MAX_FEEDBACK_FORCE;
		}
	}
	//printf("outputVel : %f\n",net_omega->MstVel[1]);
	//printf("x : %f,   y : %f,   z : %f\n",dimension_changed_pose[0],dimension_changed_pose[1],dimension_changed_pose[2]);
	//printf("x : %f,   y : %f,   z : %f\n",err[0],err[1],err[2]);
	//printf("x : %f,   y : %f,   z : %f\n",omega_->m_forcefeedback[0],omega_->m_forcefeedback[1],omega_->m_forcefeedback[2]);
	OmegaToPhantomDimension phantom2omega;
	double dimension_changed_feedback[3] = {0.0,};
	phantom2omega.PhantomToOmegaDimensionChange(omega_->m_forcefeedback[0],omega_->m_forcefeedback[1],omega_->m_forcefeedback[2],
		dimension_changed_feedback[0],dimension_changed_feedback[1],dimension_changed_feedback[2]);
	


	// remove master feedback force

	//for(int i = 0; i<DOF ; i++)
	//{
	//	dimension_changed_feedback[i] = 0.0;
	//}

	// Force Output To Omega
	if(dhdSetForceAndTorqueAndGripperForce(dimension_changed_feedback[0],dimension_changed_feedback[1],dimension_changed_feedback[2],
		0.0,0.0,0.0, 0.0) < DHD_NO_ERROR){
		printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
  		loop_state = true;
	}

	//printf("master force feedback : %lf \n" , dimension_changed_feedback[2]);

	return 1;
}



int slave_initialize(HHD &hHD_init, tel_state *state_)
{
	HDErrorInfo error;

    // Initialize the default haptic device.
    hHD_init = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        state_->system_state = SYSTEM_OFFLINE;
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

    // Start the servo scheduler and enable forces.
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        state_->system_state = SYSTEM_OFFLINE;
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

    // Phantom Device Initialize
    hdScheduleSynchronous(DeviceCalibrate, position , HD_MIN_SCHEDULER_PRIORITY);
    Calibration_state = true;
    
    // system state 
	//state_->system_state = SYSTEM_ONLINE;
    return 1;
}

int slave_popc(mst2net *net_omega, slv2net *net_phantom, SLAVEPACK *phantom_, slv_PO *s_PO, tel_state *state_)
{

    //-- POPC
    double Fs[3] = { -net_phantom->SlvForceTorque[0], -net_phantom->SlvForceTorque[1], -net_phantom->SlvForceTorque[2] }; // Force from Slave
	// double Fs[3] = {-slave_ee_force_x_, -slave_ee_force_y_, -slave_ee_force_z_}; // Force from Slave
    double Vm[3] = { net_omega->MstVel[0], net_omega->MstVel[1], net_omega->MstVel[2] }; // Master device Velocity (Position_Incremental/Dt)
    // double Vm[3] = {master_dx/Dt, master_dy/Dt, master_dz/Dt}; // Master device Velocity (Position_Incremental/Dt)


    double E_master_in[3];
    E_master_in[0] = net_omega->MstEnergy[0];
    E_master_in[1] = net_omega->MstEnergy[1];
    E_master_in[2] = net_omega->MstEnergy[2];

    static double E_slave_in[3] = { 0.0, 0.0, 0.0 };
    static double E_slave_out[3] = { 0.0, 0.0, 0.0 };

    // Below two arrays is not necessary actually. only for the data logging to analysis the result
    double E_slave_out_origin[3] = { 0.0, 0.0, 0.0 };
    double Vm_origin[3] = { Vm[0], Vm[1], Vm[2] };
	static double X_m[3] = {0.0,0.0,0.0};
	static double X_hat[3] = {0.0,0.0,0.0};
	static double X_err_prev[3] = {0.0,0.0,0.0};
	static double X_max_prev[3] = {0.0,0.0,0.0};
	static double V_prev[3] = {0.0,};
	static double V_prev_hat[3] = {0.0,};
	static double E_remain_out[3] = {0.0,0.0,0.0};
	static double E_remain_in[3] = {0.0,};
	static bool UpCheck[3] = {false,};
	static bool DownCheck[3] = {false,};
	static double X_max[3] = {0.0,};
	static double F_max[3] = {0.0,};
	static double Delta_O[3] = {0.0,};
	static double prev_err[3] = {0.0,};
	static double prev_F[3] = {0.0,};
	static double X_sd[3] = {0.0,0.0,0.0};
	static double X_s[3] = {0.0,};
	static double E_prev[3] = {0.0,};
	static double F_prev[3] = {0.0,};
	static double beta_prev[3] = {0.0,};

    for(int i =0;i<DOF;i++){
    	if (state_->POPC_Act){
      		
			switch(state_->Control_mathod)
			{
				case NORMAL_POPC :
				{
					m_PopcClass->AdmittancePOPC(E_master_in[i], E_slave_in[i], E_slave_out_origin[i], E_slave_out[i], Fs[i], Vm[i], phantom_->s_dt);
					break;
				}
				case POSITION_DRIFT_POPC :
				{
					m_PopcClass->AdmittancePOPC_PDC(E_master_in[i], E_slave_in[i], E_slave_out_origin[i], E_slave_out[i],E_prev[i], Fs[i], F_prev[i],beta_prev[i], Vm[i],
						X_m[i], X_hat[i], phantom_->s_dt);
					break;
				}
				case POSITION_DRIFT_ADVANCE_POPC :
				{
					m_PopcClass->AdmittancePOPC_PDC_advance(E_master_in[i], E_slave_in[i], E_slave_out_origin[i], E_slave_out[i],E_prev[i], Fs[i], F_prev[i],
						beta_prev[i], Vm[i], phantom_->s_dt);
					break;
				}
				default :
				{
					break;
				}
				//X_s[i] = phantom_->position[i];
				//m_PopcClass->AdmittanceSSI(E_remain_in[i],E_remain_out[i],UpCheck[i],DownCheck[i],Fs[i], prev_F[i], Vm[i], prev_err[i] ,F_max[i],X_max[i], Delta_O[i], X_sd[i], X_s[i], phantom_->s_dt);
			}
			net_phantom->SlvEnergy[i] = E_slave_in[i];	
  			
  			// Admittance Passivity Controller Output Velocity 
    		phantom_->s_vel[i] = Vm[i];
			
  		}
  		else{
  			E_master_in[i] = 0.0;
  			E_slave_in[i] = 0.0;
  			E_slave_out[i] = 0.0;
  			E_slave_out_origin[i] = 0.0;
			
  			
  			// Master Velocity
			//phantom_->s_vel[i] = Vm[i];
    		phantom_->s_vel[i] = Vm_origin[i];
  		}
		//printf("Delta O : %f\n",Delta_O[1]);
		//printf("E master in : %f, E slave out : %f\n",E_master_in[0],E_slave_out[0]);
		
		s_PO->s_Data_Es_in[i] = E_slave_in[i];
	    s_PO->s_Data_Es_out_origin[i] = E_slave_out_origin[i];
	    s_PO->s_Data_Es_out[i] = E_slave_out[i];
	    s_PO->s_Data_Em_in[i] = E_master_in[i];
		s_PO->s_Data_Epc[i] = E_prev[i];
	    s_PO->s_Data_Fs[i] = Fs[i];
	    s_PO->s_Data_Vm_origin[i] = Vm_origin[i];
	    s_PO->s_Data_Vm[i] = Vm[i];  
    }
	//printf("in : %f     out : %f    F_Max : %f    Delta_O : %f\n",E_remain_in[0],E_remain_out[0],F_max[0],Delta_O[0]);
    s_PO->s_Dt = phantom_->s_dt;
	//printf("E_master in : %f,   E slave out : %f\n",E_master_in[1],s_PO->s_Data_Es_out[1]);
    return 1;

}


int slave_phantom_gen(slv2net *net_phantom, mst2net *net_omega, SLAVEPACK *ServoData, slv_PO *s_PO, tel_state *state)
{
	// Determin sampling time
	static double Dt = 0.0;
	static clock_t old_t = clock();
	Dt = double(clock() - old_t) / 1000.0;
	old_t = clock();

	// sampling time by phantom
	Dt = 0.001;
	ServoData->s_dt = Dt;
	
	if(!(Dt>0)) return -1;

	/* Get the current location of the device (HD_GET_CURRENT_POSITION)
	We declare a vector of three doubles since hdGetDoublev returns
	the information in a vector of size 3. */
	for(int i=0;i<3;i++)
	{
		ServoData->prev[i] = ServoData->position[i];
	}
	hdGetDoublev(HD_CURRENT_POSITION, ServoData->position);

	for(int i=0;i<3;i++)
	{
		ServoData->position[i] -= init_position[i];
		ServoData->disp[i] = ServoData->position[i] - ServoData->prev[i];
	}
	
	//printf("%f\n",ServoData->position[1]);
	// time stamp by phantom
	net_phantom->time_stamp = double(clock());
	
	// Slave Passivity Observer/Passivity Controller 
	slave_popc(net_omega,net_phantom,ServoData,s_PO,state);

	// Master moving distance
	static double Xsd[3] = {0.0,0.0, 0.0};
	double m_ftsensor[3] = {0.0,};
	hduVector3Dd outputforce;

	// Impedance SSI
	static double E_remain_in[3] = {0.0,};
	static double E_remain_out[3] = {0.0,};
	static double E_remain_in2[3] = {0.0,};
	static double E_remain_out2[3] = {0.0,};
	static bool UpCheck[3] = {false,};
	static bool DownCheck[3] = {false,};
	static double prev_F[3] = {0.0,};
	static double prev_err[3] = {0.0,};
	static double F_max[3] = {0.0,};
	static double X_max[3] = {0.0,};
	static double F_max2[3] = {0.0,};
	static double X_max2[3] = {0.0,};
	static double X_sd[3] = {0.0,};
	static double X_s[3] = {0.0,};
	static double Delta_O[3] = {0.0,};
	static double Delta_O2[3] = {0.0,};
	
	// FT sensor data -- dimension change
	OmegaToPhantomDimension FTsensor2Phantom;
	if(ft_readbuf->m_head == HEADER && ft_readbuf->m_tail == TAIL){
		FTsensor2Phantom.FTsensorToPhantomDimensionChange(ft_readbuf->sFTforce[0], ft_readbuf->sFTforce[1], ft_readbuf->sFTforce[2],
			ft_sensor->force[0],ft_sensor->force[1],ft_sensor->force[2]);
	}else{
		printf("FT sensor data read fail!!!\n\n");
	}

	for(int i=0;i<3;i++){
		
		//net_phantom->ft_force[i] = ft_sensor->force[i];  
		Xsd[i] += ServoData->s_vel[i] * ServoData->s_dt;         // desired position 
		
		ServoData->force[i] = K_slave[i] * (Xsd[i] - ServoData->position[i]) - slv_damping[i] * (ServoData->disp[i]) ; 
		outputforce[i] = ServoData->force[i];
		
		if(outputforce[i] >= MAX_FEEDBACK_FORCE){
			outputforce[i] = MAX_FEEDBACK_FORCE;
		}else if(outputforce[i] <= -1 * MAX_FEEDBACK_FORCE){
			outputforce[i] = -1 * MAX_FEEDBACK_FORCE;
		}

		//slave network phantom data input 
		net_phantom->SlvPose[i] = ServoData->position[i];

		//m_ftsensor[i] = -1.0 * ServoData->force[i];;
		net_phantom->SlvForceTorque[i] = -1.0 * ServoData->force[i];
	}
	//outputforce[0] = 0.0;
	//outputforce[2] = 0.0;
	
	//printf("Energy in : %f    Energy out : %f,    Delta_O : %f  \n",E_remain_in[0],E_remain_out[0],Delta_O[0]);
	//printf("Energy in : %f    Energy out : %f,    Delta_O : %f  \n",E_remain_in[0],E_remain_out[0],Delta_O[0]);
	//printf("Xsd[%d] : %f\n",1,ServoData->s_vel[1]);
	
	//printf("outputforce : %lf \n" , outputforce[1]);

	hdSetDoublev(HD_CURRENT_FORCE,outputforce);

	return 1;
}

void print_state()
{
	int i = 1;
	//printf("ref : %f\t , cur : %f\n",SSI_data[i].e,SSI_data[i].e_prev); 
	//printf("%d\t:\t f_raw : %f\t f : %f \t E_O : %f \t  F_max : %f \t X_max : %f\t DeltaO : %f \t  %d\t  %d\n",i,SSI_data[i].f_raw,SSI_data[i].f,SSI_data[i].E_O,SSI_data[i].F_max, SSI_data[i].X_max, SSI_data[i].delta_O, SSI_data[i].mode, SSI_data[i].first_time);
}


int sharedmemory_init(tel_net &data,
					  m_omega &mst,
					  tel_state &state,
					  mst2net &net_mst,
					  mst2net &net_mst_delay,
					  slv2net &net_slv,
					  slv2net &net_slv_delay,
					  mst_PO &m_PO,
					  slv_PO &s_PO,
					  SLAVEPACK &slvServoData,
					  DeviceData &gServoDeviceData)
{
	// teleoperation state
	//state = new tel_state;

	// Master
	//mst = new m_omega;
	//net_mst = new mst2net;
	//net_mst_delay = new mst2net;
	//m_PO = new mst_PO;
	
	//data = new tel_net;
	
	// Slave
	//slvServoData = (SLAVEPACK*)malloc(sizeof(SLAVEPACK)); 
	//gServoDeviceData = (DeviceData*)malloc(sizeof(DeviceData));
	//net_slv = new slv2net;
	//net_slv_delay = new slv2net;
	//s_PO = new slv_PO;

	// Initalize Data structureture
	TELEOPERATION_STATE_INIT(&state);
	TELOEPERATION_MST_INIT(&net_mst);
	TELOEPERATION_MST_INIT(&net_mst_delay);
	MASTER_PO_INIT(&m_PO);

	TELEOPERATION_SLV_INIT(&net_slv);
	TELEOPERATION_SLV_INIT(&net_slv_delay);
	SLAVE_PO_INIT(&s_PO);


	
	// master data structure initialize
	mst.m_dt = 0.0;
	
	// system parameter intialize
	state.POPC_Act = true;
	state.m_save = true;
	state.TimeDelay_Check = true;
	state.virtual_mass_spring_on = false;
	
	/*
	Control Method
	NORMAL_POPC - NORMAL ADMITTANCE COTNROL
	POSITION_DRIFT_POPC - JORDI POSITION DRIFT COMPENSATOR VER.
	POSITION_DRIFT_ADVANCE_POPC - VINAY POSITION DRIFT COMPENSATOR VER.
	*/

	//state.Control_mathod = POSITION_DRIFT_ADVANCE_POPC;
	//state.Control_mathod = POSITION_DRIFT_POPC;
	//state.Control_mathod = NORMAL_POPC;
	state.Control_mathod = NORMAL_POPC;


	state.Delay_time = 0;
	state.Packet_loss = 100;
	
	// system parameter intialize


	state.v_mass = 0.001;
	state.v_spring = 0.2;
	state.v_damping = 0.01;
	// system state
	state.system_state = SYSTEM_LOADING;

	// POPC Library class 
	m_PopcClass = new KTBioLabLib::AlpopcLib;

	// POPC Library class Intialize
	m_PopcClass->POPC_Intialize(0.0, 0.0, 0.0, 0.0, 0.0);
	
	// RateMode Library class Intialize
	m_RatemodeClass = new KTBioLabLib::AlteleoperationLib;
	
	//m_RatemodeClass->RateMode_Initalize(0.7,0.5,10.0,0.7,30.0);
	m_RatemodeClass->RateMode_Initalize(0.3,0.5,10.0,0.7,15.0);
	

	// Moving Average Data
	moving_aver_x = new double[moving_aver_num];
	moving_aver_y = new double[moving_aver_num];
	moving_aver_z = new double[moving_aver_num];


	m_PopcClass->SSIstructure_Init(&SSI_data[0]);
	m_PopcClass->SSIstructure_Init(&SSI_data[1]);
	m_PopcClass->SSIstructure_Init(&SSI_data[2]);

	for(int i = 0;i<moving_aver_num;i++)
	{
		moving_aver_x[i] = 0.0;
		moving_aver_y[i] = 0.0;
		moving_aver_z[i] = 0.0;
	}

	// Shared memory Intialize
	ft_readbuf = new FTvalue;
	ft_sensor = new FTSENSOR;
	FTSENSOR_INIT(ft_sensor);
	
	int PAGE_SIZE = sizeof(FTvalue);

	hMapping = OpenFileMapping(
		FILE_MAP_READ | FILE_MAP_WRITE,
		FALSE,
		OBJECT_NAME
		);

	if (hMapping == NULL){
		printf("Shared Memory Problem : Can not open memory!!!\n");
		exit(0);
		return 0;
	}

	ft_readbuf = (FTvalue*)MapViewOfFile(
		hMapping,
		FILE_MAP_ALL_ACCESS,
		0,
		0,
		PAGE_SIZE);

	if (ft_readbuf == NULL){
		printf("Shared Memory Problem : Can not read memory!!!\n");
		exit(0);
		return 0;
	}

	/*
	// between master and network program data structure intialize
	net_mst->m_ControlMode = 0;
	net_mst->m_indexing = false;

	// Passivity Observer data structure intialize
	m_PO->m_Dt = 0.0;
	
	for(int i =0;i < DOF;i++){
		// master data structure initialize
		mst->m_pose[i] = 0.0;
		mst->m_prevpose[i] = 0.0;
		mst->m_vel[i] = 0.0;

		// between master and network program data structure intialize
		net_mst->MstVel[i] = 0.0;
		net_mst->MstEnergy[i] = 0.0;

		// Passivity Observer data structure intialize
		m_PO->m_Data_Em_in[i] = 0.0;
		m_PO->m_Data_Em_out_origin[i] = 0.0;
		m_PO->m_Data_Em_out[i] = 0.0;
		m_PO->m_Data_Es_in[i] = 0.0;
		m_PO->m_Data_Fs_origin[i] = 0.0;
		m_PO->m_Data_Fs[i] = 0.0;
		m_PO->m_Data_Fvms[i] = 0.0;
		m_PO->m_Data_Vm[i] = 0.0;
	}
	for(int i =0;i<6;i++){
		// master data structure initialize
		mst->m_forcefeedback[i] = 0.0;
		mst->m_energy[i] = 0.0;
	}
	*/
	return 1;
}

void exit_all_program()
{
	UnmapViewOfFile(ft_readbuf);
	CloseHandle(hMapping);

	// POPC Library class 
	delete(m_PopcClass);

	// Rate Mode Library
	delete(m_RatemodeClass);

	delete(moving_aver_x);
	delete(moving_aver_y);
	delete(moving_aver_z);
	delete(ft_sensor);

}

