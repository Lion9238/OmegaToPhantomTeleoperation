#include "KTBioLabLib.hpp"

namespace KTBioLabLib
{
	AlHapticLib::AlHapticLib()
	{
		// SFA Initial
		planeStiffness = 0.0;
		incremental_value = 0.001;
		input_k = 1.2;
		penetrationDistance_previous = 0.0;
		penetrationDistance = 0.0;
		displayed_stiffness = 0.0;
		value_for_penetration = 0.0;
		first_time_counter = 0.0;
		SFA_Initialize_fail = INITIALIZE_FAIL;
		prevDirection = 0;
		_prevstiffness = 0.0;
		Fe = 0.0;
		f1 = 0.0;
		f = 0.0;

	}

	AlHapticLib::~AlHapticLib()
	{

	}
	void AlHapticLib::HapticSFA_Initalize(double _incremental_value, double _k_small)
	{

	    //   | y
		//	 |_______
		//	 /        x 
		//  / z

		input_k = _k_small;
		incremental_value = _incremental_value;
	}
	
	void AlHapticLib::private_SFA(double position)
	{
		Fe = f;
		penetrationDistance_previous = penetrationDistance;
		penetrationDistance = 1.0 * (position);
       
		double k = planeStiffness;
		double x = penetrationDistance;
		
		f1 = k * penetrationDistance;
		
		if (penetrationDistance < penetrationDistance_previous)
		{
			if (displayed_stiffness < 0)
			{
				value_for_penetration = value_for_penetration + incremental_value ;
			}
			else if (displayed_stiffness > 0 && displayed_stiffness < planeStiffness)
			{
				if ((planeStiffness - displayed_stiffness) > 1)
				{
					value_for_penetration = value_for_penetration - 0.01 * (planeStiffness - displayed_stiffness);
				}
				else
				{
					value_for_penetration = value_for_penetration - incremental_value;
				}
			}
			else if (displayed_stiffness >= planeStiffness)
			{
				value_for_penetration = value_for_penetration + incremental_value;
			}
			if (Fe >= f1)
			{
				f = (k_small * penetrationDistance + (value_for_penetration));
			}
			else
			{
				f = (k_small * penetrationDistance + (value_for_penetration));
			}
			if (first_time_counter < 1)
			{
				f = k * penetrationDistance;
			}
		}
		else if (penetrationDistance > penetrationDistance_previous)
		{
			if (first_time_counter < 1)
			{
				f = (k_small * penetrationDistance + value_for_penetration);
			}
			else
			{
				f = (k_small * penetrationDistance + value_for_penetration);
			}
		    first_time_counter = first_time_counter + 1;
		}
		
		
		displayed_stiffness = (f) / (penetrationDistance);
	}

	void AlHapticLib::SFA_reset()
	{
		penetrationDistance_previous = 0;
		penetrationDistance = 0;
		displayed_stiffness = 0;
		value_for_penetration = 0;
		first_time_counter = 0;
		Fe = 0.0;
		f1 = 0.0;
		f = 0.0;

	}

	void AlHapticLib::HapticSFA(int Direction, double _stiffness, double _position, double &output_force)
	{

		if(prevDirection != Direction || _stiffness != _prevstiffness)
		{
			SFA_reset();
		}

		if(_stiffness < input_k)
		{
			planeStiffness = _stiffness;
			k_small = _stiffness;
		}
		else if(_stiffness >= input_k)
		{
			planeStiffness = _stiffness;
			k_small = input_k;
		}
		

		if(Direction == SFA_UP)
		{
			if ((_position <= -value_for_penetration))
			{
				private_SFA(_position);
				output_force = f;
			}
			else
			{
				f = 0;
				output_force = f;
				displayed_stiffness = 0;
				value_for_penetration = 0;
				first_time_counter = 0;
				
			}
		}
		else if(Direction == SFA_DOWN)
		{
			if ((_position >= -value_for_penetration))
			{
				private_SFA(_position);
				output_force = f;
			}
			else
			{
				f = 0;
				output_force = f;
				displayed_stiffness = 0;
				value_for_penetration = 0;
				first_time_counter = 0;
				
			}
		}
		else
		{
			output_force = f;
		}

		prevDirection = Direction;
		_prevstiffness = _stiffness;
	}
	

	AlteleoperationLib::AlteleoperationLib()
	{

		RMC_Initialize_fail = INITIALIZE_FAIL;
		SC_Initialize_fail = INITIALIZE_FAIL;
		VKG_Initialize_fail = INITIALIZE_FAIL;
		TOSFA_Initialize_fail = INITIALIZE_FAIL;
	}

	AlteleoperationLib::~AlteleoperationLib()
	{

	}

	void AlteleoperationLib::TOSuccesiveForceAugment_Initalize(double Stiffness,double ActForce,double incremental,double k)
	{
		planeStiffness = Stiffness;
		ActiveForce = ActForce;
		incremental_value = incremental;
		k_small = k;
		TOSFA_Initialize_fail = INITIALIZE_COMPLETE;
	}

	int AlteleoperationLib::TOSuccesiveForceAugment(bool indexing,double ForceAug,double Pose,double &Velo, double Master_Dt,
		double &Xmd, double &Xsd,double &Xdiff, double &displayed_stiffness, double &FeedbackForce)
	{
		if(TOSFA_Initialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;
		/*
		// RATE Mode 
		if(ControlMode == RATE_MODE_SFA)
		{
			Boundary = 0.0;
			
			if(Pose >= -X0 && Pose <= X0)
			{
				Boundary = Beta * Velo;
				Velo = Boundary;
			}
			else if(Pose < -X0)
			{
				Velo = Alpha * (Pose + X0);
			}
			else if(Pose > X0)
			{
				Velo = Alpha * (Pose - X0[i]);	
			}
		}
		*/
		// SFA Mode
		if(indexing)
		{
			Xmd += (Velo * Master_Dt);
			
			Xdiff = Xmd - Xsd;
			// Get the position of the device.
			if (fabs(ForceAug) > ActiveForce){
			//if(gServoDeviceData.m_buttonState){
				
				//if((-1 * gServoDeviceData.m_devicePosition[i]) >= value_for_penetration[i]){
				if(Xmd - Xsd > 0.0)
				{
					//SFAdirection[i] = Up;
					Xdiff = Xmd - Xsd - Xdiff;
				}
				else
				{
					//SFAdirection[i] = Down;
					Xdiff = Xmd - Xsd + Xdiff;
					
				}

				//DiffForce[i] = ForceAug[i] - DiffForce[i];
				//Xdiff[i] = Xmd[i] - Xsd[i];
				penetrationDistance_previous = penetrationDistance;
				penetrationDistance = Xdiff;
				
				
				
				//penetrationDistance[i] = 1.0 * (gServoDeviceData.m_devicePosition[i]);
				 

				// Start of Pressing Path
				if (penetrationDistance < penetrationDistance_previous){
					// fix the planeStiffness
					//&& displayed_stiffness[i] < planeStiffness
					if(Xdiff > 0.0)
					{
						if (displayed_stiffness >= 0 && displayed_stiffness < planeStiffness)
						{
							//printf("+\n");
							value_for_penetration = value_for_penetration + incremental_value;
						}
						else 
						{
							//printf("-\n");
							value_for_penetration = value_for_penetration - incremental_value;
						}
						FeedbackForce = k_small * penetrationDistance + value_for_penetration;
						//FeedbackForce[i] = ForceFeedback_[i] + value_for_penetration[i];

						if (first_time_counter < 1)
						{
							FeedbackForce = planeStiffness * penetrationDistance;			// f = kx
							//FeedbackForce[i] = ForceFeedback_[i];
						}
					}
					else
					{
						if (displayed_stiffness >= 0 && displayed_stiffness < planeStiffness)
						{
							//printf("-\n");
							value_for_penetration = value_for_penetration - incremental_value;
						}
						else 
						{
							//printf("+\n");
							value_for_penetration = value_for_penetration + incremental_value;
						}
						FeedbackForce = k_small * penetrationDistance + value_for_penetration;
						//FeedbackForce[i] = ForceFeedback_[i] + value_for_penetration[i];
						
						if (first_time_counter < 1)
						{
							FeedbackForce = planeStiffness * penetrationDistance;			// f = kx
							//FeedbackForce[i] = ForceFeedback_[i];
						}
							
					}
				}

				// End of Pressing Path

				// Start of Releasing Path
				else if (penetrationDistance >= penetrationDistance_previous)
				{
					//printf("Releasing\n");
					FeedbackForce = k_small * penetrationDistance + value_for_penetration;
					//FeedbackForce[i] = ForceFeedback_[i] + value_for_penetration[i];
					
					first_time_counter = first_time_counter + 1;
				}
				// End of Releasing Path
				
				displayed_stiffness = (FeedbackForce) / (penetrationDistance); 
				
			}
			else
			{

				
				//Xdiff[i] = 0.0;
				FeedbackForce = 0.0;
				displayed_stiffness = 0;
				value_for_penetration = 0;
				first_time_counter = 0;
			}
			
			/*
			if(ControlMode == RATE_MODE_SFA)
			{
				FeedbackForce = FeedbackForce - (Kp * err + Kd * derr);
			}
			*/
			///// check this line
			//ForceFeedback_[0] = FeedbackForce[0];
			//ForceFeedback_[1] = FeedbackForce[1];
			//ForceFeedback_[2] = FeedbackForce[2];


		}
		else
		{
			Xmd = 0.0;
			Xsd = 0.0;
			Xdiff = 0.0;
		}

		return INITIALIZE_COMPLETE;
	}

	void AlteleoperationLib::RateMode_Initalize(double A, double B,double kp, double kd, double x0)
	{
		Alpha = A;
		Beta = B;
		Kp = kp;
		Kd = kd;
		X0 = x0;

		RMC_Initialize_fail = INITIALIZE_COMPLETE;
	}
	
	int AlteleoperationLib::RateModeControl(bool Indexing,double err,double derr, double Pose, 
		double &outputVel, double &OutputForce)
	{
		if(RMC_Initialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;


		if (Indexing){
			OutputForce = OutputForce + Kp * err + Kd * derr;
			
			if(Pose >= -X0 && Pose <= X0)
			{
				Boundary = Beta * (outputVel);
				outputVel = Boundary;
			}
			else if(Pose < -X0)
			{
				//outputVel = Alpha * (Pose + X0);
				outputVel = Alpha * (Pose);
			}
			else if(Pose > X0)
			{
				//outputVel = Alpha * (Pose - X0);
				outputVel = Alpha * (Pose);
			}
		}
		else{
			OutputForce = Kp * err + Kd * derr;
			outputVel = 0.0;
		}

		

		
		return INITIALIZE_COMPLETE;

	}

	void AlteleoperationLib::SwitchingControl_Initalize(double _ControllerFrontMax, double _ControllerBehindMax, double _MaxSpeed_, double _MinSpeed_,
		double _MaxDistance_, double _MinDistance_, double _X_DeadZone, double _StackVeloDeadZone)
	{
		ControllerFrontMax = _ControllerFrontMax;
		ControllerBehindMax = _ControllerBehindMax;
		MaxSpeed_ = _MaxSpeed_;
		MinSpeed_ = _MinSpeed_;
		MaxDistance_ = _MaxDistance_;
		MinDistance_ = _MinDistance_;
		X_DeadZone = _X_DeadZone;
		StackVeloDeadZone = _StackVeloDeadZone;

		SC_Initialize_fail = INITIALIZE_COMPLETE;
	}

	int AlteleoperationLib::SwitchingControl(bool Indexing, int SwitchingMode, double X_value, double &Output_Value, double sampling_time)
	{
		if(SC_Initialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;

		if(SwitchingMode == SWITCHING_VELO)
		{
			
			if(X_value >= -X_DeadZone && X_value <= X_DeadZone)
			{
				X_value = 0.0;
			}		

			if(X_value < ControllerFrontMax)
			{
				X_value = ControllerFrontMax;
			}
			else if(X_value > ControllerBehindMax)
			{
				X_value = ControllerBehindMax;
			}

			double value = MinSpeed_ + (X_value - ControllerBehindMax) * (MaxSpeed_ - MinSpeed_) / (ControllerFrontMax - ControllerBehindMax);
			
			if(value > MaxSpeed_)
			{
				value = MaxSpeed_;
			}
			else if(value < MinSpeed_)
			{
				value = MinSpeed_;
			}

			if(Indexing)
				Output_Value = value;
			else
				Output_Value = 0.0;
			
			
		}
		else if(SwitchingMode == SWITCHING_POSE)
		{
			static double StackVelo = 0.0;

			if(X_value >= -X_DeadZone && X_value <= X_DeadZone)
			{
				X_value = 0.0;
			}		

			if(X_value < ControllerFrontMax)
			{
				X_value = ControllerFrontMax;
			}
			else if(X_value > ControllerBehindMax)
			{
				X_value = ControllerBehindMax;
			}

			double value = MinDistance_ + (X_value - ControllerBehindMax) * (MaxDistance_ - MinDistance_) / (ControllerFrontMax - ControllerBehindMax);
			double VeloValue = MinSpeed_ + (X_value - ControllerBehindMax) * (MaxSpeed_ - MinSpeed_) / (ControllerFrontMax - ControllerBehindMax);

			
			if(Indexing)
			{
				if(VeloValue > MaxSpeed_)
				{
					Output_Value = MaxSpeed_;
				}
				else if(VeloValue < MinSpeed_)
				{
					Output_Value = MinSpeed_;
				}
				
				if(StackVelo >= value - StackVeloDeadZone && StackVelo <= value + StackVeloDeadZone)
				{
					Output_Value = 0.0;
					
				}
				else if(StackVelo < value)
				{
					StackVelo = StackVelo + VeloValue * sampling_time;

					Output_Value = VeloValue;
					
				}
				else if(StackVelo > value)
				{
					StackVelo = StackVelo - VeloValue * sampling_time;

					Output_Value = -VeloValue;
					
				}
			}
			else
			{
				StackVelo = 0.0;
				Output_Value = 0.0;
			}
		}

		return INITIALIZE_COMPLETE;
	}

	void AlteleoperationLib::VariableKgainControl_Initalize(double _Min_Dist, double _Max_Kp, double _Min_Kp, double _Max_Force, double _Min_Force, double _max_speed)
	{
		Max_Kp = _Max_Kp;
		Min_Kp = _Min_Kp;
		Max_Force = _Max_Force;
		Min_Force = _Min_Force;
		Min_Dist = _Min_Dist;
		max_speed = _max_speed;
		VKG_Initialize_fail = INITIALIZE_COMPLETE;
	}

	int AlteleoperationLib::VariableKgainControl(double front_sensor_value, double rear_sensor_value,double &Ouput_Force, double sampling_time)
	{
		if(VKG_Initialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;

		// variable K gain Tunning
		static double prev_obs_dist = 0.0;
		static double prev_diff_dist = 0.0;
		static double prev_dist = 0.0;
		static double diff_dist = 0.0;

	    double obs_dist = (Min_Dist - rear_sensor_value) - (Min_Dist - front_sensor_value);
	    double diff_obs_dist = prev_obs_dist - obs_dist;

	    if(obs_dist == prev_dist)
	        diff_dist = prev_diff_dist;
	    else
	        diff_dist = diff_obs_dist;

	    double Gain = 0.0;
	    if(diff_dist >= 0)
	        Gain = Min_Kp;
	    else if(diff_dist > -max_speed * sampling_time && diff_dist < 0)
	        Gain = -1.0/(max_speed * sampling_time) * (Max_Kp - Min_Kp) * diff_dist + Min_Kp;
	    else
	        Gain = Max_Kp;


	    // Ouput Value
		Ouput_Force = Gain * obs_dist;
		if(Ouput_Force > Max_Force)
			Ouput_Force = Max_Force;
		else if(Ouput_Force < Min_Force)
			Ouput_Force = Min_Force;


	    prev_obs_dist = obs_dist;
	    prev_dist = obs_dist;
	    prev_diff_dist = diff_dist;

	    return INITIALIZE_COMPLETE;
	}

	AlpopcLib::AlpopcLib()
	{
		MIN_FORCE_ERROR = 0.0;
		MIN_VEL_ERROR = 0.0;
			
		M_vms = 0.0;
		K_vms = 0.0;
		B_vms = 0.0;

		O_Fs_d = 0.0;
		O_prev_Fs_d = 0.0;
		O_Fm_proxy = 0.0;
		O_prev_Fm_proxy = 0.0;
		O_prev_Fm = 0.0;
		O_prev_error = 0.0;
		O_prev_Fs = 0.0;

		MOVING_EVER_INIT = INITIALIZE_FAIL;
		Intialize_fail = INITIALIZE_FAIL;
			
			
	}

	AlpopcLib::~AlpopcLib()
	{

	}

	void AlpopcLib::POPC_Intialize(double _MIN_FORCE_ERROR, double _MIN_VEL_ERROR, double _M_vms, double _K_vms, double _B_vms)
	{
		MIN_FORCE_ERROR = _MIN_FORCE_ERROR;
		MIN_VEL_ERROR = _MIN_VEL_ERROR;

		M_vms = _M_vms;
		K_vms = _K_vms;
		B_vms = _B_vms;

		Intialize_fail = INITIALIZE_COMPLETE;
		//std::cout << "INTIALIZING" << std::endl;
	}

	//Impedance POPC
	int AlpopcLib::ImpedancePOPC(double& E_s_in, double& E_m_in, double& E_m_out_origin, double& E_m_out, 
		double& Fm, double& F_vms, double Xm, double& X_vms, double Vm, double& V_vms, double Dt){
		// MinForceFiltering(Fm);
		// MinVelocityFiltering(Vm);
			
		if(Intialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;

		double E_m_out_prev = E_m_out; // E_m_out(k-1)

		//Energy Update
		E_m_in = InputEnergyUpdate(E_m_in,Fm,Vm,Dt); // E_m_in(k)
		E_m_out = OutputEnergyUpdate(E_m_out,Fm,Vm,Dt); // E_m_out(k)

		//Passivity Controller
		double F_origin = Fm;
		E_m_out_origin = E_m_out;

		if(E_m_out + E_s_in < 0.0){
			if(Vm*Vm > 0.0){
				E_m_out += Fm*Vm*Dt; // Restoring the Energy
				Fm = (E_s_in + E_m_out)/(Vm*Dt); // Force modification to make E_m_out(k) same with E_s_in(k)
				E_m_out = -E_s_in; // Energy(E_m_out(k)) Update with modified Force
			}
			//printf("Em_in: %.6f, Em_out: %.6f, Em_out2: %.6f, Es_in: %.6f, Fm: %.6f, Vm: %.6f, Fmm: %.6f\n",E_m_in,E_m_out_origin, E_m_out, E_s_in, F_origin, Vm, Fm);
		}

		// Virtual Mass Spring Damping
		double A_vms = 0.0;
		// static double Vc = Vm;
		// static double Xc = 0.0;

		double dx = X_vms - Xm; 
		double dv = V_vms - Vm;

		A_vms = (Fm - K_vms*dx - B_vms*V_vms)/M_vms;
		V_vms += A_vms * Dt;
		X_vms += V_vms * Dt;

		F_vms = K_vms*dx + B_vms*dv;

		return INITIALIZE_COMPLETE;
	}
	//Impedance POPC
	int AlpopcLib::ImpedancePOPC_PDC(double& E_s_in, double& E_m_in, double& E_m_out_origin, double& E_m_out, double &E_m_remain, 
		double& Fm, double& F_vms, double Xm, double& X_vms, double Vm, double& V_vms, double Dt){
		// MinForceFiltering(Fm);
		// MinVelocityFiltering(Vm);
		if(Intialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;

		double F_delayed = Fm;
		double E_m_out_prev = E_m_out; // E_m_out(k-1)
		double alpha = 0.0;
		//Energy Update
		E_m_in = InputEnergyUpdate(E_m_in,F_delayed,Vm,Dt); // E_m_in(k)
		E_m_out = OutputEnergyUpdate(E_m_out,F_delayed,Vm,Dt); // E_m_out(k)
		

		//Passivity Controller
		double F_origin = F_delayed;
		E_m_out_origin = E_m_out;	
		if(E_m_out + E_s_in + E_m_remain < 0.0){
			if(Vm*Vm > 0.0){
				
				alpha = ((E_s_in + E_m_out + E_m_remain)/(Vm*Vm*Dt));
				E_m_out += F_delayed*Vm*Dt; // Restoring the Energy
				//Fm = (E_s_in + E_m_out + E_m_remain)/(Vm*Dt); // Force modification to make E_m_out(k) same with E_s_in(k)
				Fm = F_delayed + alpha*Vm;
				//E_m_out = -E_s_in; // Energy(E_m_out(k)) Update with modified Force
				E_m_out -= Fm*Vm*Dt;
				E_m_remain -= Dt*alpha*Vm*Vm;
			}else{
				alpha = 0.0;
			}
			//printf("Em_in: %.6f, Em_out: %.6f, Em_out2: %.6f, Es_in: %.6f, Fm: %.6f, Vm: %.6f, Fmm: %.6f\n",E_m_in,E_m_out_origin, E_m_out, E_s_in, F_origin, Vm, Fm);
		}else{
			alpha = 0.0;
		}
		
		// Virtual Mass Spring Damping
		double A_vms = 0.0;
		// static double Vc = Vm;
		// static double Xc = 0.0;

		double dx = X_vms - Xm; 
		double dv = V_vms - Vm;

		A_vms = (Fm - K_vms*dx - B_vms*V_vms)/M_vms;
		V_vms += A_vms * Dt;
		X_vms += V_vms * Dt;

		//F_vms = K_vms*dx + B_vms*dv;
		F_vms = Fm;
		return INITIALIZE_COMPLETE;
	}
	//Impedance POPC
	int AlpopcLib::ImpedancePOPC_modified(double& E_s_in, double& E_m_in, double& E_m_out_origin, double& E_m_out, double &E_m_remain, 
		double& Fm, double& F_vms, double Xm, double& X_vms, double Vm, double& V_vms, double Dt){

		
		// MinForceFiltering(Fm);
		// MinVelocityFiltering(Vm);
		if(Intialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;

		double E_m_out_prev = E_m_out; // E_m_out(k-1)

		//Energy Update
		E_m_in = InputEnergyUpdate(E_m_in,Fm,Vm,Dt); // E_m_in(k)
		E_m_out = OutputEnergyUpdate(E_m_out,Fm,Vm,Dt); // E_m_out(k)

		//Passivity Controller
		double F_origin = Fm;
		E_m_out_origin = E_m_out;
		E_m_remain = 0.0;
		//E_m_remain -= E_m_out - E_s_in;
		E_m_remain = 0.0;
		if(E_m_out + E_s_in < E_m_remain){
			if(Vm*Vm > 0.0){
				E_m_out += Fm*Vm*Dt; // Restoring the Energy
				Fm = (E_s_in + E_m_out - E_m_remain)/(Vm*Dt); // Force modification to make E_m_out(k) same with E_s_in(k)
				//E_m_out = -E_s_in; // Energy(E_m_out(k)) Update with modified Force
				E_m_out -= Fm*Vm*Dt;
			}
			//printf("Em_in: %.6f, Em_out: %.6f, Em_out2: %.6f, Es_in: %.6f, Fm: %.6f, Vm: %.6f, Fmm: %.6f\n",E_m_in,E_m_out_origin, E_m_out, E_s_in, F_origin, Vm, Fm);
		}
		
		// Virtual Mass Spring Damping
		double A_vms = 0.0;
		// static double Vc = Vm;
		// static double Xc = 0.0;

		double dx = X_vms - Xm; 
		double dv = V_vms - Vm;

		A_vms = (Fm - K_vms*dx - B_vms*V_vms)/M_vms;
		V_vms += A_vms * Dt;
		X_vms += V_vms * Dt;

		F_vms = K_vms*dx + B_vms*dv;

		return INITIALIZE_COMPLETE;
	}

	int AlpopcLib::ImpedancePOPC_NonVMS(double& E_s_in, double& E_m_in, double& E_m_out_origin, double& E_m_out, 
		double& Fm, double& F_modified, double Vm, double Dt){

		// MinForceFiltering(Fm);
		// MinVelocityFiltering(Vm);

		if(Intialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;

		double E_m_out_prev = E_m_out; // E_m_out(k-1)

		//Energy Update
		E_m_in = InputEnergyUpdate(E_m_in,Fm,Vm,Dt); // E_m_in(k)
		E_m_out = OutputEnergyUpdate(E_m_out,Fm,Vm,Dt); // E_m_out(k)

		//Passivity Controller
		double F_origin = Fm;
		F_modified = Fm;
		E_m_out_origin = E_m_out;

		if(E_m_out + E_s_in < 0.0){
			if(Vm*Vm > 0.0){
				E_m_out += Fm*Vm*Dt; // Restoring the Energy
				F_modified = (E_s_in + E_m_out)/(Vm*Dt); // Force modification to make E_m_out(k) same with E_s_in(k)
				E_m_out = -E_s_in; // Energy(E_m_out(k)) Update with modified Force
			}
		}

		return INITIALIZE_COMPLETE;
	}

	int AlpopcLib::OBGImpedancePOPC_NonVMS(double& E_s_in, double& E_m_in, double& E_m_out_origin, double& E_m_out, 
		double& Fm, double Vm, double O_error, double Dt){
		// MinForceFiltering(Fm);
		// MinVelocityFiltering(Vm);
		

		if(Intialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;

		O_Fs_d = Fm;
		if(O_Fs_d > 0.001)
		{
			if(O_error >= O_prev_error){
				O_Fm_proxy = O_prev_Fm_proxy + (O_Fs_d - O_prev_Fs_d);
				Fm = O_prev_Fm + (O_Fs_d - O_prev_Fs_d);
			}
			else if(O_error < O_prev_error){
				O_Fm_proxy = O_prev_Fm_proxy - fabs(fabs(O_Fs_d) - fabs(O_prev_Fs_d));
				Fm = O_prev_Fm - fabs(fabs(O_Fs_d) - fabs(O_prev_Fs_d));
			}
			else{
				O_Fm_proxy = O_prev_Fm_proxy;
				Fm = O_prev_Fm;
			}

			if (O_Fm_proxy < 0){
				O_Fm_proxy = 0.0;
				Fm = 0.0;
			}

		}
		else if(O_Fs_d < -0.001)
		{
			if(O_error <= O_prev_error){
				O_Fm_proxy = O_prev_Fm_proxy + (O_Fs_d - O_prev_Fs_d);
				Fm = O_prev_Fm + (O_Fs_d - O_prev_Fs_d);
			}
			else if(O_error > O_prev_error){
				O_Fm_proxy = O_prev_Fm_proxy + fabs(fabs(O_Fs_d) - fabs(O_prev_Fs_d));
				Fm = O_prev_Fm + fabs(fabs(O_Fs_d) - fabs(O_prev_Fs_d));
			}
			else{
				O_Fm_proxy = O_prev_Fm_proxy;
				Fm = O_prev_Fm;
			}

			if (O_Fm_proxy > 0){
				O_Fm_proxy = 0.0;
				Fm = 0.0;
			}
		}
		else
		{
			O_Fm_proxy = 0.0;
			Fm = 0.0;
		}
		//O_Fm_proxy = O_Fm_proxy 



		double E_m_out_prev = E_m_out; // E_m_out(k-1)

		//Energy Update
		E_m_in = InputEnergyUpdate(E_m_in,Fm,Vm,Dt); // E_m_in(k)
		E_m_out = OutputEnergyUpdate(E_m_out,Fm,Vm,Dt); // E_m_out(k)

		//Passivity Controller
		double F_origin = Fm;
		E_m_out_origin = E_m_out;
		
		if(E_m_out + E_s_in < 0.0){
			if(Vm*Vm > 0.0){
				E_m_out += Fm*Vm*Dt; // Restoring the Energy
				Fm = (E_s_in + E_m_out)/(Vm*Dt); // Force modification to make E_m_out(k) same with E_s_in(k)
				E_m_out = -E_s_in; // Energy(E_m_out(k)) Update with modified Force
			}
		}

		O_prev_error = O_error;
		O_prev_Fm_proxy = O_Fm_proxy;
		O_prev_Fm = Fm;
		O_prev_Fs = O_Fs_d;

		return INITIALIZE_COMPLETE;
	}


	//Admittance POPC
	int AlpopcLib::AdmittancePOPC(double& E_m_in, double& E_s_in, double& E_s_out_origin, double& E_s_out, double& F, double& V, double Dt){
		// MinForceFiltering(F);
		// MinVelocityFiltering(V);

		if(Intialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;

		double E_s_out_prev = E_s_out;

		//Energy Update
		E_s_in = InputEnergyUpdate(E_s_in,F,V,Dt);
		E_s_out = OutputEnergyUpdate(E_s_out,F,V,Dt);
			
		//Passivity Controller
		double V_origin = V;
		E_s_out_origin = E_s_out;
		double beta = 0.0;
		

		if(E_s_out + E_m_in < 0.0){
			if(F*F > 0.0){
				E_s_out += F*V*Dt;
				V = (E_m_in + E_s_out)/F/Dt;
				E_s_out = -E_m_in;
			}
			//printf("Es_in: %.6f, Es_out: %.6f, Es_out2: %.6f, Em_in: %.6f, Fs: %.6f, Vs: %.6f, Vsm: %.6f\n",E_s_in,E_s_out_origin, E_s_out, E_m_in, F, V_origin, V);
		}

		return INITIALIZE_COMPLETE;
	}

	void AlpopcLib::ImpedanceNONPOPC(double& E_m_in, double& E_m_out, double& Fm, double Vm,  double Dt){
		// MinForceFiltering(Fm);
		// MinVelocityFiltering(Vm);

		//Energy Update
		E_m_in = InputEnergyUpdate(E_m_in,Fm,Vm,Dt); // E_m_in(k)
		E_m_out = OutputEnergyUpdate(E_m_out,Fm,Vm,Dt); // E_m_out(k)

	}
	
	//Admittance POPC - Position Drift Compensator
	int AlpopcLib::AdmittancePOPC_PDC(double& E_m_in, double& E_s_in, double& E_s_out_origin, double& E_s_out, double & E_s_previous, 
		double &F, double &F_prev, double &beta_prev, double &V, double &X_sd, double &X_hat, double Dt){
		// MinForceFiltering(F);
		// MinVelocityFiltering(V);
		
		if(Intialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;

		double E_s_out_prev = E_s_out;
		double V_delayed = V;

		//Energy Update
		E_s_in = InputEnergyUpdate(E_s_in, F, V_delayed, Dt);
		E_s_out = OutputEnergyUpdate(E_s_out, F, V_delayed, Dt);

		//Passivity Controller
		double V_origin = V_delayed;
		E_s_out_origin = E_s_out;
		double beta = 0.0;
		double X_err = 0.0;
		double X_max = 0.0;
		X_hat += V_delayed * Dt;
		//X_sd += V_delayed * Dt;
		if(E_m_in + E_s_out + E_s_previous < 0.0){
			if(F*F > 0.0){

				//E_s_out += F * (V_delayed) * Dt;
				beta = ((E_m_in + E_s_out + E_s_previous)/(F*F*Dt));
				//V = (E_m_in + E_s_out)/F/Dt;
				V = V_delayed + beta * F;
				//E_s_out = -E_m_in;
				E_s_out -= F*V*Dt;
				X_sd += V*Dt;
				E_s_previous -= Dt*beta*F*F;
			}else{
				beta = 0.0;
			}
		}else{
			if(F*F > 0.0){
				
				double X_err = X_sd - X_hat;
				double X_max = (E_m_in + E_s_out + E_s_previous)/F;	
				
				double sign = 0.0;
				if(X_hat - X_sd > 0){
					sign = 1.0;
				}else{
					sign = -1.0;
				}

				if(fabs(X_max) > fabs(X_err)){
					beta = sign * fabs(X_err) / (F * Dt);
					printf("X err\n");
				}else{
					beta = sign * fabs(X_max) / (F * Dt);
					printf("X max\n");
				}

				V = V_delayed + beta*F;
				X_sd += V*Dt;

				//E_s_previous -= Dt*beta*F*F;
			}
			beta = 0.0;
		}

		beta_prev = beta;
		F_prev = F;

		return INITIALIZE_COMPLETE;
	}
	
	int AlpopcLib::AdmittancePOPC_PDC_advance(double& E_m_in, double& E_s_in, double& E_s_out_origin, double& E_s_out, double & E_s_previous, 
		double& F, double& F_prev, double& beta_prev, double& V, double Dt){
		// MinForceFiltering(F);
		// MinVelocityFiltering(V);

	

		if(Intialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;

		double E_s_out_prev = E_s_out;
		double V_delayed = V;
		double Vad = beta_prev * F_prev;

		//Energy Update
		E_s_in = InputEnergyUpdate(E_s_in,F,V_delayed + Vad,Dt);
		E_s_out = OutputEnergyUpdate(E_s_out,F,V_delayed + Vad,Dt);
		

		//Passivity Controller
		double V_origin = V_delayed;
		E_s_out_origin = E_s_out;
		double beta = 0.0;
		double X_err = 0.0;
		double X_max = 0.0;
		
		if(E_s_out + E_m_in + E_s_previous < 0.0){
			if(F*F > 0.0){

				E_s_out += F * (V_delayed + Vad) * Dt;
				beta = ((E_m_in + E_s_out + E_s_previous)/(F*F*Dt));
				//V = (E_m_in + E_s_out)/F/Dt;
				V = V_delayed + Vad + beta * F;
				//E_s_out = -E_m_in;
				E_s_out -= F*V*Dt;
				E_s_previous -= Dt*beta*F*F;

			}else{
				beta = 0.0;
			}
		}else{
			beta = 0.0;	
		}
		
		beta_prev = beta;
		F_prev = F;

		return INITIALIZE_COMPLETE;
	}
	/*
	int AlpopcLib::ImpedanceSSI(double &U_Storage_Energy_in, double &U_Storage_Energy_out,double &D_Storage_Energy_in, double &D_Storage_Energy_out, bool &UpCheck, bool &DownCheck, double& F, double &prev_F, double X, double& prev_err, 
		double &Up_F_max, double &Up_X_max,double &Do_F_max, double &Do_X_max, double &Delta_O, double Dt){
		// MinForceFiltering(F);
		// MinVelocityFiltering(V);
		
		if(Intialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;

		//Passivity Controller
		//double V_origin = V;
		double beta = 0.0;

		double V_err = 0.0;
		double V_max = 0.0;
		
		double Force = F;
		
		double err = X;
		
		static int first_cycle = 0;

		
		if(err >= 0.0)
		{
			if(first_cycle == 0){
				//Pressing
				if(err - prev_err > 0.0)
				{
					if(Up_X_max > err){
						Up_F_max = F;
						Up_X_max = err;
					}
					DownCheck = false;
				}
				// Releasing
				else{

					U_Storage_Energy_out += F * (err - prev_err);
					DownCheck = true;
				}
				first_cycle = 1;
			}
			else{
				if(UpCheck == true){
					//if(Do_X_max*Do_X_max > 0.0 && D_Storage_Energy_out != 0.0)
					Delta_O =  2.0 * (D_Storage_Energy_out - (Do_F_max * Do_X_max / 2.0)) / (Do_X_max) ;
						//Delta_O =  2.0 * (D_Storage_Energy_out + D_Storage_Energy_in) / (Do_X_max);
					//else
						//Delta_O = 0.0;
					Do_F_max = 0.0;
					Do_X_max = 0.0;
					D_Storage_Energy_in = 0.0;
					D_Storage_Energy_out = 0.0;
					UpCheck = false;
				}
				F = Force + Delta_O *0.01;
				// Pressing
				if(err - prev_err > 0.0)
				{
					
					if(Up_X_max > err){
						Up_F_max = F;
						Up_X_max = err;
					}
					U_Storage_Energy_in += F * (err - prev_err);
					DownCheck = false;
				
				}
				// Releaseing
				else if(err - prev_err < 0.0)
				{
					U_Storage_Energy_out += F * (err - prev_err);
					DownCheck = true;
				}
			}
			
		}
		else if(err < 0.0){
			
			if(first_cycle == 0){
				//Pressing
				if(err - prev_err < 0.0)
				{
					if(Do_X_max < err){
						Do_F_max = F;
						Do_X_max = err;
					}
					UpCheck = false;
				}
				// Releasing
				else{
					
					D_Storage_Energy_out += F * (err - prev_err);
					UpCheck = true;
				}
				first_cycle = 1;
			}
			else{
				if(DownCheck == true)
				{
					//if(Up_X_max*Up_X_max > 0.0)
					Delta_O = 2.0 * (U_Storage_Energy_out - (Up_F_max * Up_X_max / 2.0)) / (Up_X_max);
						//Delta_O = 2.0 * (U_Storage_Energy_out + U_Storage_Energy_in) / (Up_X_max);
					
					Up_F_max = 0.0;
					Up_X_max = 0.0;
					U_Storage_Energy_in = 0.0;
					U_Storage_Energy_out = 0.0;
					DownCheck = false;
				}
				F = Force - Delta_O * 0.01;
				if(err - prev_err < 0.0)
				{	
					
					if(Do_X_max < err){
						Do_F_max = F;
						Do_X_max = err;
					}					
					D_Storage_Energy_in += F * (err -  prev_err);
				
					UpCheck = false;	
				}
				else if(err - prev_err > 0.0)
				{
					F = Force;
					D_Storage_Energy_out += F * (err - prev_err);
					UpCheck = true;
				}
			}
		}
		
		
		prev_err = err;
		prev_F = F;
		return INITIALIZE_COMPLETE;
	}
	*/

	void AlpopcLib::SSIstructure_Init(SSIstructure  *data){
	  data->mode = READY;
	  data->E_S = 0.0;
	  data->E_D = 0.0;
	  data->delta_O = 0.0;
	  data->X_max = 0.0;
	  data->F_max = 0.0;
	  data->E_O = 0.0;
	  data->ref_pose = 0.0;
	  data->cur_pose = 0.0;
	  data->e = 0.0;
	  data->e_dot = 0.0;
	  data->e_prev = 0.0;
	  data->first_time = 0;
	  data->f = 0.0;
	  data->f_raw = 0.0;
	  data->prev_delta_O = 0.0;
	  data->Dt = 0.001;
	}

	double AlpopcLib::ImpedanceSSI(SSIstructure &input)
	{
	  input.f = input.f_raw;
	  input.e_prev = input.e;
	  //double e_prev = input.e;
	  input.e = (input.ref_pose - input.cur_pose);
	  input.e_dot = (input.e - input.e_prev) / input.Dt;
	  double alpha = 0.1;
	  double max_force = 1.5;
	  if(input.e_dot >=  0.0)
	  {
		if(input.first_time == 0)
		{
		  switch (input.mode){
			case READY :
			  input.mode = UP_PRESSING; 
			  break;
			case UP_PRESSING :
			  input.F_max = input.f;
			  input.X_max = input.e;
			  if(input.F_max > max_force)
				input.F_max = max_force;

			  //input.E_O = 0.0;
			  break;
			case DOWN_PRESSING :
			  input.first_time = 1;
			  input.mode = UP_RELEASING;
			  input.E_O += input.f * input.e_dot * input.Dt;
			  if(input.X_max != 0.0)
				input.delta_O = 2.0 / input.X_max  * (input.E_O + (input.X_max * input.F_max) / 2.0);
			  else
				input.delta_O = 0.0;
			  break;
			default :
			  break;
		  }
		}
		else{
		  switch(input.mode){
			case UP_RELEASING : 
			  input.f = input.f_raw + alpha * input.prev_delta_O;
			  input.E_O += (input.f) * input.e_dot * input.Dt;
			  if(input.X_max != 0)
				input.delta_O = 2.0 / input.X_max  * (input.E_O + (input.X_max * input.F_max) / 2.0);
			  else
				input.delta_O = 0.0;

			  if(input.f > 0.0){
				input.mode = UP_PRESSING;
			  }

			  break;
			  
			case UP_PRESSING :
			  input.f = input.f_raw + alpha * input.delta_O;
			  input.F_max = input.f;
			  input.X_max = input.e;
			  if(input.F_max > max_force)
				input.F_max = max_force;
				
			  
			  input.prev_delta_O = input.delta_O;
			  input.E_O = 0.0;
			  break;

			case DOWN_RELEASING :
			  input.mode = DOWN_PRESSING;
			  input.f = input.f_raw - alpha * input.delta_O;
			  input.F_max = input.f;
			  input.X_max = input.e;
			  if(input.F_max > max_force)
				input.F_max = max_force;
			  input.prev_delta_O = -input.delta_O;
			  input.E_O = 0.0;
			  break;
			  
			case DOWN_PRESSING :
			  input.mode = UP_RELEASING;
			  input.f = input.f_raw + alpha * input.prev_delta_O;
			  input.E_O += (input.f) * input.e_dot * input.Dt;
			  if(input.X_max != 0.0)
				input.delta_O = 2.0 / input.X_max  * (input.E_O + (input.X_max * input.F_max) / 2.0);
			  else
				input.delta_O = 0.0;

			  break;
			default :
			  break;
		  }
		}
	  }
	  else if(input.e_dot <  0.0)
	  {
		if(input.first_time == 0)
		{
		  switch(input.mode){
			case READY :
			  input.mode = DOWN_PRESSING;
			  break;
			case DOWN_PRESSING :
			  input.F_max = input.f;
			  input.X_max = input.e;
			  if(input.F_max > max_force)
				input.F_max = max_force;
			  input.E_O = 0.0;
			  break;
			case UP_PRESSING :
			  input.first_time = 1;
			  input.mode = DOWN_RELEASING;
			  input.E_O += input.f * input.e_dot * input.Dt;
			  
			  if(input.X_max != 0.0)
				input.delta_O = 2.0 / input.X_max  * (input.E_O + (input.X_max * input.F_max) / 2.0);
			  else
				input.delta_O = 0.0;
			  break;
			default :
			  break;
		  }
		}
		else{
		  switch(input.mode){
			case DOWN_PRESSING :
			  input.f = input.f_raw - alpha * input.delta_O;
			  input.F_max = input.f;
			  input.X_max = input.e;
			  if(input.F_max > max_force)
				input.F_max = max_force;
			  input.E_O = 0.0;
			  input.prev_delta_O = -input.delta_O;
			  break;

			case UP_RELEASING :
			  input.mode = UP_PRESSING;
			  input.f = input.f_raw + alpha * input.delta_O;
			  input.F_max = input.f;
			  input.X_max = input.e;
			  if(input.F_max > max_force)
				input.F_max = max_force;
			  input.prev_delta_O = input.delta_O;
			  input.E_O = 0.0;
			  	  
			  break;
			  
			case UP_PRESSING :
			  input.mode = DOWN_RELEASING;
			  input.f = input.f_raw + alpha * input.prev_delta_O;
			  input.E_O += (input.f) * input.e_dot * input.Dt;
			  if(input.X_max != 0 )
				input.delta_O = 2.0 / input.X_max  * (input.E_O + (input.X_max * input.F_max) / 2.0);
			  else
				input.delta_O = 0.0;
			  break;
			 
			case DOWN_RELEASING :
			  input.f = input.f_raw + alpha * input.prev_delta_O;
			  input.E_O += (input.f) * input.e_dot * input.Dt;
			  if(input.X_max != 0)
				input.delta_O = 2.0 / input.X_max  * (input.E_O + (input.X_max * input.F_max) / 2.0);
			  else
				input.delta_O = 0.0;

			  if(input.f < 0.0)
			  {
				  input.mode = DOWN_PRESSING;
			  }
			  break;
			  
			default :
			  break;
		  }
		}
	  }else{
		input.f = input.f_raw;
      }
  
	  return input.f;
	  //return 0.0;
	}
	
	int AlpopcLib::AdmittanceSSI(double &Storage_Energy_in, double &Storage_Energy_out, bool &UpCheck, bool &DownCheck, double& F, double &prev_F, double& V, double& prev_err, 
		double &F_max, double &X_max, double &Delta_O, double &A_sd, double &A_s, double Dt){
		// MinForceFiltering(F);
		// MinVelocityFiltering(V);

		if(Intialize_fail == INITIALIZE_FAIL)
			return INITIALIZE_FAIL;


		//Passivity Controller
		double V_origin = V;
		double beta = 0.0;

		double V_err = 0.0;
		double V_max = 0.0;
		
		A_sd += (V * Dt);
		double err = A_sd - A_s;
		//printf("%f\n",V);
		if(err > 0)
		{
			
			if(UpCheck == true)
			{
				if(F_max*F_max > 0.0)
					Delta_O = (2.0 * (Storage_Energy_out + Storage_Energy_in) + F_max * X_max) / F_max - X_max;
				else
					Delta_O = 0.0;
				//A_sd += Delta_O;
				F_max = 0.0;
				X_max = 0.0;
				Storage_Energy_in = 0.0;
				Storage_Energy_out = 0.0;
				UpCheck = false;
			}
			
			if(err - prev_err > 0)
			{
				
				if( F > 0){
					F_max = F;
					X_max = err;
					Storage_Energy_in += F * (err - prev_err);
				}else{
					F_max = 0.0;
					X_max = 0.0;
				}
				DownCheck = false;
			}
			else
			{
				if( F > 0 && err > 0){
					Storage_Energy_out += F * (err - prev_err);
					DownCheck = true;
				}
				
			}	
			V = err + Delta_O;
		}
		else
		{
			if(DownCheck == true)
			{
				if(F_max*F_max > 0.0)
					Delta_O = (2.0 * (Storage_Energy_out + Storage_Energy_in) + F_max * X_max) / F_max - X_max;
				else
					Delta_O = 0.0;
				//A_sd += Delta_O;
				F_max = 0.0;
				X_max = 0.0;
				Storage_Energy_in = 0.0;
				Storage_Energy_out = 0.0;
				DownCheck = false;
			}
			
			if(err - prev_err < 0)
			{	
				if(F < 0){
					F_max = F;
					X_max = err;
					Storage_Energy_in += F * (err - prev_err);
				}else{
					F_max = 0.0;
					X_max = 0.0;
				}

				
				UpCheck = false;
			}
			else
			{
				if(F < 0){
					Storage_Energy_out += F * (err - prev_err);
					UpCheck = true;
				}
				
			}
			V = err - Delta_O;
		}
		//printf("%f \n",V);
		prev_err = err;
		prev_F = F;
		return INITIALIZE_COMPLETE;
	}

	void AlpopcLib::AdmittanceNONPOPC(double& E_m_in, double& E_s_in, double& E_s_out_origin, double& E_s_out, double& F, double& V, double Dt){
		// MinForceFiltering(F);
		// MinVelocityFiltering(V);

		//Energy Update
		E_s_in = InputEnergyUpdate(E_s_in,F,V,Dt);
		E_s_out = OutputEnergyUpdate(E_s_out,F,V,Dt);

	}

	void AlpopcLib::MinForceFiltering(double& F){
		if(fabs(F) < MIN_FORCE_ERROR){
			F = 0.0;
		}
	}
	void AlpopcLib::MinVelocityFiltering(double& V){
		if(fabs(V) < MIN_VEL_ERROR){
			V = 0.0;
		}
	}
	void AlpopcLib::MovingEverageFilter_Init(double moving_aver_num_){
		moving_aver_num = moving_aver_num_;
		moving_counter = 0;
		moving_counter_checker = false;
		MOVING_EVER_INIT = INITIALIZE_COMPLETE;

		moving_aver = new double[moving_aver_num];
	}
	int AlpopcLib::MovingEverageFilter(double value,double& output_value){
		
		if(MOVING_EVER_INIT == INITIALIZE_FAIL)
			return -1;

		// moving average filter
		if(moving_counter != moving_aver_num-2){
			moving_aver[moving_counter] = double(value);
			moving_counter++;
		}else{
			moving_aver[moving_counter] = double(value);
			moving_counter_checker = true;
			moving_counter = 0;
		}
		double value_tmp_ = 0;
		if(moving_counter_checker == true){
			for(int i=0;i<moving_aver_num;i++){
				value_tmp_ += moving_aver[i];
			}
			output_value = value_tmp_ / double(moving_aver_num);
		}
		return INITIALIZE_COMPLETE;
	}

	double AlpopcLib::InputEnergyUpdate(double previous_energy, double F, double V, double Dt){
		double updated_energy;
		double P; //Power;
		P = F*V;
		if(P < 0){
			updated_energy = previous_energy - P*Dt;
		}
		else{
			updated_energy = previous_energy;
		}
		return updated_energy;
	}

	double AlpopcLib::OutputEnergyUpdate(double previous_energy, double F, double V, double Dt){
		double updated_energy;
		double P; //Power;
		P = F*V;
		if(P > 0){
			updated_energy = previous_energy - P*Dt;
		}
		else{
			updated_energy = previous_energy;
		}
		return updated_energy;
	}


	// Impedance Causality
	inline double AlpopcLib::Modified_Fm(double Fm, double Vm, double alpha){
		double modified_fm = Fm - alpha*Vm;
		return modified_fm;	
	}

	// Admittance Causality
	inline double AlpopcLib::Modified_Vs(double Vs, double Fs, double beta){
		double modified_vs = Vs - beta*Fs;
		return modified_vs;
	}

	// Damping Factor for Impedance Causality
	double AlpopcLib::Damping_Alpha(double E_out, double E_in, double Vm, double Dt){
		double alpha = 0.0;
		if(Vm != 0.0){
			if(E_out > E_in){
				alpha = (E_out - E_in)/(Dt * Vm*Vm);	
			}
		}
		return alpha;
	}

	// Damping Factor for Admittance Causality
	double AlpopcLib::Damping_Beta(double E_out, double E_in, double Fs, double Dt){
		double beta = 0.0;
		if(Fs != 0.0){
			if(E_out > E_in){
				beta = (E_out - E_in)/(Dt * Fs*Fs);
			}
		}
		return beta;
	}

} // namespace KTBioLabLib