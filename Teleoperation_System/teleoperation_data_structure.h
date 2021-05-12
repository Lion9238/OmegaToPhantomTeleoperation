

// Teleoperation System Omega(Master) to Phantom(Slave) Program
// State Data Type, Master Data Type, Slave Data Type, Passivity Oberver(Master, Slave) Data Type
// This Program tested in Windows 10
// Omega3, Omega6 to Phnatom Premium 3.0


#include <stdio.h>
#include <stdlib.h>

//#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
//#include <HDU/hduMatrix.h>

#define REFRESH_INTERVAL    0.1   //  seconds
#define LINEAR_VISCOSITY   30.0   //  N/(m/s)
#define MAX_FRICTION_FORCE 0.75
// #define MAX_FRICTION_FORCE 0.00001
#define FRICTION_SAT_REGION 0.05
#define ANGULAR_VISCOSITY   0.04  // Nm/(rad/s)

#define SERIAL_NUMBER_OMEGA3 11415 // Omega3
#define SERIAL_NUMBER_OMEGA6 11359 // Omega6
#define SERIAL_NUMBER_SIGMA7 3030 // Sigma7

#define DOF 3

//#define SYSTEM_OFFLINE 1120
//#define SYSTEM_ONLINE 1321
//#define SYSTEM_LOADING 3212

const int SYSTEM_OFFLINE = 1120;
const int SYSTEM_ONLINE = 1321;
const int SYSTEM_LOADING = 3212;

// Control Method
#define NORMAL_POPC 4123
#define POSITION_DRIFT_POPC 4523
#define POSITION_DRIFT_ADVANCE_POPC 4524
#define FOUR_CHANEL_COMMUNICATION 5231 


// Shared Memory
#define HEADER 0x31b
#define TAIL 0x31c


#define OBJECT_NAME _T("FT SHARED MEMORY")

#define MAX_FEEDBACK_FORCE 6.0

#pragma pack(push,1)

// Slave Phantom data structure
// Phantom Premium (Slave)
typedef struct _PACK{
	unsigned int sequence;
	hduVector3Dd position;
//	hduVector3Dd position_prev;
	hduVector3Dd prev;
	hduVector3Dd force;
//	hduVector3Dd force_prev;
	hduVector3Dd disp;
//	hduVector3Dd s_disp;
	hduVector3Dd s_vel;
	double s_dt;
	//hduVector3Dd force;
}SLAVEPACK;

typedef struct
{
	hduVector3Dd m_devicePosition;
	hduVector3Dd m_deviceVelocity;
	hduVector3Dd m_deviceGimbalAngle;
	hduVector3Dd m_deviceJointAngle;
	HDErrorInfo m_error;
	HDboolean m_buttonState;
} DeviceData;


typedef struct TELEOPERATION_STATE
{
	bool POPC_Act;
	bool m_save;
	bool virtual_mass_spring_on;
	bool TimeDelay_Check;
	int Control_mathod;
	int Delay_time;
	int Packet_loss;
	double v_mass;
	double v_spring;
	double v_damping;
	int system_state;
}tel_state;

typedef struct TELEOPERATION_FTSENSOR_DATA{
	double force[3];
	double torque[3];
}FTSENSOR;

typedef struct {
	int m_head;
	double sFTforce[3];
	double sFTtorque[3];
	int m_tail;
}FTvalue;

typedef struct TELEOPERATION_MASTER_NETWORK
{
	unsigned int m_ControlMode;
	bool m_indexing;
	double MstVel[DOF];
	double MstEnergy[DOF];
	double time_stamp;
}mst2net;

typedef struct TELEOPERATION_SLAVE_NETWORK
{
	double SlvPose[DOF];
	double SlvForceTorque[DOF];
	double SlvEnergy[DOF];
	double time_stamp;
}slv2net;

typedef struct MASTER_PASSIVITY_OBSERVER
{
	double m_Data_Em_in[DOF];
	double m_Data_Em_out_origin[DOF];
	double m_Data_Em_out[DOF];
	double m_Data_Es_in[DOF];
	double m_Data_Epc[DOF];
	double m_Data_Fs_origin[DOF];
	double m_Data_Fs[DOF];
	double m_Data_Fvms[DOF];
	double m_Data_Vm[DOF];
	double m_Dt;
}mst_PO;

typedef struct SLAVE_PASSIVITY_OBSERVER
{
	double s_Data_Es_in[DOF];
	double s_Data_Es_out_origin[DOF];
	double s_Data_Es_out[DOF];
	double s_Data_Em_in[DOF];
	double s_Data_Epc[DOF];
	double s_Data_Fs[DOF];
	double s_Data_Vm_origin[DOF];
	double s_Data_Vm[DOF];
	double s_Dt;
}slv_PO;

typedef struct MASTER_OMEGA
{
	bool m_indexing;
	double m_forcefeedback[DOF];
	double m_pose[DOF];
	double m_prevpose[DOF];
	double m_vel[DOF];
	double m_energy[DOF];
	double m_dt;
}m_omega;

typedef struct SLAVE_PHANTOM
{
	double s_pose[3];
	double s_prevpose[3];
	double s_vel[3];
	double s_force[3];
	double s_energy[3];
	double s_dt;
}s_phantom;

typedef struct TELEOPERATION_STRUCTURE
{
	tel_state state;
	mst2net mst_net;
	slv2net slv_net;
}tel_net;

int TELEOPERATION_STATE_INIT(tel_state *state)
{
	state->POPC_Act = false;
	state->m_save = false;
	state->virtual_mass_spring_on = false;
	state->TimeDelay_Check = false;
	state->Delay_time = 0;
	state->Packet_loss = 0;
	state->v_mass = 0.0;
	state->v_spring = 0.0;
	state->v_damping = 0.0;
	state->system_state = SYSTEM_OFFLINE;
	return 1;
}

int TELOEPERATION_MST_INIT(mst2net *net)
{
	net->m_ControlMode = 0;
	net->m_indexing = false;
	for(int i =0;i<DOF;i++){
		net->MstVel[i] = 0.0;
		net->MstEnergy[i] = 0.0;
	}
	net->time_stamp = 0.0;
	return 1;
}

int TELEOPERATION_SLV_INIT(slv2net * net)
{
	for(int i =0;i<DOF;i++){
		net->SlvPose[i] = 0.0;
		net->SlvForceTorque[i] = 0.0;
		net->SlvEnergy[i] = 0.0;
	}
	net->time_stamp;
	return 1;
}

int MASTER_PO_INIT(mst_PO *PO)
{
	for(int i=0;i<DOF;i++){
		PO->m_Data_Em_in[i] = 0.0;
		PO->m_Data_Em_out_origin[i] = 0.0;
		PO->m_Data_Em_out[i] = 0.0;
		PO->m_Data_Es_in[i] = 0.0;
		PO->m_Data_Fs_origin[i] = 0.0;
		PO->m_Data_Fs[i] = 0.0;
		PO->m_Data_Fvms[i] = 0.0;
		PO->m_Data_Vm[i] = 0.0;
	}
	PO->m_Dt = 0.0;
	return 1;
}

int SLAVE_PO_INIT(slv_PO *PO)
{
	for(int i=0;i<DOF;i++){
		PO->s_Data_Es_in[i] = 0.0;
		PO->s_Data_Es_out_origin[i] = 0.0;
		PO->s_Data_Es_out[i] = 0.0;
		PO->s_Data_Em_in[i] = 0.0;
		PO->s_Data_Fs[i] = 0.0;
		PO->s_Data_Vm_origin[i] = 0.0;
		PO->s_Data_Vm[i] = 0.0;
	}
	PO->s_Dt = 0.0;
	return 1;
}

int MASTER_OMEGA_INIT(m_omega *omega)
{
	omega->m_indexing = false;
	for(int i=0;i<DOF;i++){
		omega->m_forcefeedback[i] = 0.0;
		omega->m_pose[i] = 0.0;
		omega->m_prevpose[i] = 0.0;
		omega->m_vel[i] = 0.0;
		omega->m_energy[i] = 0.0;
	}
	omega->m_dt = 0.0;
	return 1;
}

int SLAVE_PHANTOM_INIT(s_phantom *phantom)
{
	for(int i =0;i<3;i++)
	{
		phantom->s_pose[i] = 0.0;
		phantom->s_prevpose[i] = 0.0;
		phantom->s_vel[i] = 0.0;
		phantom->s_force[i] = 0.0;
		phantom->s_energy[i] = 0.0;
	}
	phantom->s_dt;
	return 1;
}

int FTSENSOR_INIT(FTSENSOR *ftsensor)
{
	for(int i=0;i<3;i++){
		ftsensor->force[i] = 0.0;
		ftsensor->torque[i] = 0.0;
	}
	return 1;
}
