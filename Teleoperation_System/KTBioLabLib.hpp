#include <stdio.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include <string>
#include <stdlib.h>

// Robot Control Basic Parameter
#define LeftPhantomMode 1
#define RightPhantomMode 2

// MODE (Manipulator Robot Mode / Mobile Robot Mode)
#define MANIPULATOR_ROBOT_MODE 1
#define MOBILE_ROBOT_MODE 2
#define STOP_MODE 1000


#define SFAACTIVE 1 
#define RATE_MODE_SFA 11
#define FTSENSOR_FEEDBACK 2 
#define NONFTSENSOR_FEEDBACK 3
#define RATE_MODE 4
#define DIRECT_TELEOPERATION 5

#define SWITCHING_POSE 1
#define SWITCHING_VELO 2


#define INITIALIZE_FAIL -1
#define INITIALIZE_COMPLETE 1

#define SFA_UP 1
#define SFA_DOWN -1

#define READY 0
#define UP_PRESSING 10
#define UP_RELEASING 20
#define DOWN_PRESSING 11
#define DOWN_RELEASING 22

typedef struct SuccessiveStiffnessIncrementStructure{
  /*
  mode
  0 : Ready
  1 : Up Pressing
  2 : Up Releaseing
  3 : Down Pressing
  4 : Down Releasing

  */
  int mode;
  double E_S;
  double E_D;
  double delta_O;
  double prev_delta_O;
  double X_max;
  double F_max;
  double E_O;
  double ref_pose;
  double cur_pose;
  double e;
  double e_dot;
  double e_prev;
  int first_time;
  double f;
  double f_raw;
  double Dt;
}SSIstructure;


struct RobotMode{
	int UserInterface;
	int Mode;
	int ControlMode;
	int MR_SwitchingMode;
	double MR_MaxSpeed;
};

// KOREATECH Biorobotics Lab. Teleoperation Algorithm lib
// Made by Chan Il Lee (email : yi11280595@gmail.com)
// Ver 0.1 Initial Version
namespace KTBioLabLib
{
	class AlHapticLib
	{
	public:
		AlHapticLib();
		~AlHapticLib();

		
		void HapticSFA_Initalize(double _incremental_value, double _k_small);
		/*
			
		
		*/
		void HapticSFA(int Direction, double _stiffness, double _position, double &output_force);
		/*

		*/
	private:
		void private_SFA(double position);
		void SFA_reset();
		
	private:
		
		// SFA Initial
		double planeStiffness;
		double incremental_value;
		double k_small;
		double input_k;
		double penetrationDistance_previous;
		double penetrationDistance;
		double displayed_stiffness;
		double value_for_penetration;
		double first_time_counter;
		int SFA_Initialize_fail;

		double Fe;
		double f1;
		double f;


		int prevDirection;
		double _prevstiffness;

	};

	class AlteleoperationLib
	{
	public:
		AlteleoperationLib();
		~AlteleoperationLib();

		void TOSuccesiveForceAugment_Initalize(double Stiffness,double ActForce,double incremental,double k);
		/*

		
		*/
		int TOSuccesiveForceAugment(bool indexing,double ForceAug,double Pose,double &Velo, double Master_Dt,
			double &Xmd, double &Xsd,double &Xdiff, double &displayed_stiffness, double &FeedbackForce);
		/*

		*/
		
		

		void RateMode_Initalize(double A, double B,double kp, double kd, double x0);
		/*
			// Master send command to slave
			x : position
			x0 : boundary
			command = A * (x - x0) + B * x_dot
			zero hold force = Kp * err + Kd * derr
		*/
		void SwitchingControl_Initalize(double _ControllerFrontMax, double _ControllerBehindMax, double _MaxSpeed_, double _MinSpeed_,
			double _MaxDistance_, double _MinDistance_, double _X_DeadZone, double _StackVeloDeadZone);
		/*
			Calbirate Master Device's Position to Velocity
			ControllerFrontMax = Master Device's Position Max
			ControllerBehindMax = Master Device's Position Min
			MaxSpeed, MinSpeed = Velocity mode Max & Min velo
			MaxDistance, MinDistance = position mode Max & Min pose
			StackVeloDeadZone = recommand 0.1 or 0.01  
		*/
		void VariableKgainControl_Initalize(double _Min_Dist, double _Max_Kp, double _Min_Kp, double _Max_Force, double _Min_Force, double _max_speed);
		/*
			u = Variable K gain * err;

		*/
		int SwitchingControl(bool Indexing, int SwitchingMode, double X_value, double &Ouput_Value, double sampling_time);
		
		int VariableKgainControl(double front_sensor_value, double rear_sensor_value,double &Ouput_Force, double sampling_time);
		
		int RateModeControl(bool Indexing,double err,double derr, double Pose, 
			double &outputVel, double &OutputForce);
	private:
		// Rate Mode Control Parameter
		double Boundary;
		double Alpha;
		double Beta;
		double Kp;
		double Kd;
		double X0;
		int RMC_Initialize_fail;

		// Switching Control
		double ControllerFrontMax;
		double ControllerBehindMax;
		double MaxSpeed_;
		double MinSpeed_;
		double MaxDistance_;
		double MinDistance_;
		double X_DeadZone;
		double StackVeloDeadZone;
		int SC_Initialize_fail;

		// Variable K gain Algorithm
		double Max_Kp;
		double Min_Kp;
		double Max_Force;
		double Min_Force;
		double Min_Dist;
		double max_speed;
		int VKG_Initialize_fail;

		// SFA Initial
		double planeStiffness;
		double ActiveForce;
		double incremental_value;
		double k_small;
		double penetrationDistance_previous;
		double penetrationDistance;
		double displayed_stiffness;
		double value_for_penetration;
		double first_time_counter;
		int TOSFA_Initialize_fail;
	};

	class AlpopcLib
	{
	public:
		AlpopcLib();
		~AlpopcLib();

		void POPC_Intialize(double _MIN_FORCE_ERROR, double _MIN_VEL_ERROR, double _M_vms, double _K_vms, double _B_vms);
		/*
			m*x_ddot + b * x_dot + k * x
			_M_vms : virtual mass
			_K_vms : virtual spring
			_B_vms : virtual damping  
		*/
		// 4 Channels Architectures
		int FourChannels_TDPC_A(double& E_s_in, double& E_m_in, double& E_m_out_origin, double& E_m_out, 
			double& Fm, double& F_vms, double Xm, double& X_vms, double Vm, double& V_vms, double Dt);
		/*
			4 Channels Architectures  - Impedance Passivity Controller
		*/

		int ImpedancePOPC(double& E_s_in, double& E_m_in, double& E_m_out_origin, double& E_m_out, 
			double& Fm, double& F_vms, double Xm, double& X_vms, double Vm, double& V_vms, double Dt);
		/*
			Basic Master Controller - Impedance Passivity Controller
		*/
		int ImpedancePOPC_PDC(double& E_s_in, double& E_m_in, double& E_m_out_origin, double& E_m_out, double &E_m_remain,
			double& Fm, double& F_vms, double Xm, double& X_vms, double Vm, double& V_vms, double Dt);
		/*
			Position Drift Compensator Master Controller - Impedance Passivity Controller 
		*/
		int ImpedancePOPC_modified(double& E_s_in, double& E_m_in, double& E_m_out_origin, double& E_m_out, double &E_m_remain,
			double& Fm, double& F_vms, double Xm, double& X_vms, double Vm, double& V_vms, double Dt);
		/*
			Modified Master Controller - Impedance Passivity Controller 
		*/
		int ImpedancePOPC_NonVMS(double& E_s_in, double& E_m_in, double& E_m_out_origin, double& E_m_out, 
			double& Fm, double& F_modified, double Vm, double Dt);
		/*
			Not Using Virtual Mass spring 
			Original Impedance POPC
		*/
		int AlpopcLib::OBGImpedancePOPC_NonVMS(double& E_s_in, double& E_m_in, double& E_m_out_origin, double& E_m_out, 
			double& Fm, double Vm, double O_error, double Dt);
		/*
			Observer Based Gradient controller 
		*/
		int AdmittancePOPC(double& E_m_in, double& E_s_in, double& E_s_out_origin, double& E_s_out, double& F, double& V, double Dt);
		/*
			Basic Slave Controller - Admittance Passvity Controller
		*/
		int AdmittancePOPC_PDC(double& E_m_in, double& E_s_in, double& E_s_out_origin, double& E_s_out,double & E_s_previous, double& F, double& F_prev, double& beta_prev, double& V,double &X_sd, double &X_hat, 
			double Dt);
		/*
			Slave Position Drift Compensator - Admittance Passvity Controller
		*/
		int AdmittancePOPC_PDC_advance(double& E_m_in, double& E_s_in, double& E_s_out_origin, double& E_s_out, double & E_s_previous, 
			double& F, double& F_prev, double& beta_prev, double& V, double Dt);
		/*
			Slave Position Drift Compensator Advanced Version - Admittance Passvity Controller
		*/
		void SSIstructure_Init(SSIstructure  *data);
		double ImpedanceSSI(SSIstructure &input);
		//int ImpedanceSSI(double &U_Storage_Energy_in, double &U_Storage_Energy_out,double &D_Storage_Energy_in, double &D_Storage_Energy_out, bool &UpCheck, bool &DownCheck, 
		//	double& F, double &prev_F, double X, double& prev_err, double &Up_F_max, double &Up_X_max,double &Do_F_max, double &Do_X_max, double &Delta_O, double Dt);
		/*
			Impedance Type SSI(Successive Stiffness Increment)
		*/
		int AdmittanceSSI(double &Storage_Energy_in, double &Storage_Energy_out, bool &UpCheck, bool &DownCheck, double& F , double &prev_F, double& V, double& prev_err, 
			double &F_max, double &X_max, double &Delta_O, double &A_sd, double &A_s, double Dt);
		/*
			Admittance Type SSI(Successive Stiffness Increment)
		*/
		void MinForceFiltering(double& F);
		void ImpedanceNONPOPC(double& E_m_in, double& E_m_out, double& Fm, double Vm,  double Dt);
		void AdmittanceNONPOPC(double& E_m_in, double& E_s_in, double& E_s_out_origin, double& E_s_out, double& F, double& V, double Dt);
		void MinVelocityFiltering(double& V);
		void MovingEverageFilter_Init(double moving_aver_num_);
		int MovingEverageFilter(double value, double &output_value);
		double InputEnergyUpdate(double previous_energy, double F, double V, double Dt);
		double OutputEnergyUpdate(double previous_energy, double F, double V, double Dt);
		inline double Modified_Fm(double Fm, double Vm, double alpha);
		inline double Modified_Vs(double Vs, double Fs, double beta);
		double Damping_Alpha(double E_out, double E_in, double Vm, double Dt);
		double Damping_Beta(double E_out, double E_in, double Fs, double Dt);

	public:
		int c;
	private:
		double MIN_FORCE_ERROR;
		double MIN_VEL_ERROR;

		double M_vms;
		double K_vms;
		double B_vms;

		//OBG controller Parameter
		double O_Fs_d;
		double O_prev_Fs_d;
		double O_Fm_proxy;
		double O_prev_Fm_proxy;
		double O_prev_Fm;
		double O_prev_error;
		double O_prev_Fs;

		// MOVING EVERAGE FILTER
		bool MOVING_EVER_INIT;
		int moving_aver_num;
		int moving_counter;
		bool moving_counter_checker;
		double *moving_aver;

		int Intialize_fail;
	};
} // namespace KTBioLabLib