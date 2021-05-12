
// Teleoperation System Omega(Master) to Phantom(Slave) Program
// This Program tested in Windows 10
// Omega3, Omega6 Master to Phantom Slave Teleoperation System
// Made by Chanil Lee
#include "teleoperation_algo.h"

// Omega lib 
#include <dhdc.h>

// Phantom lib
//#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
//#include <HDU/hduMatrix.h>

// Omega lib 
#include "dhdc.h"
#include "drdc.h"



// Delay Data Saving
extern std::vector<mst2net> omega_data_stack;
extern std::vector<slv2net> phantom_data_stack;


extern FTvalue *ft_readbuf;
extern FTSENSOR *ft_sensor;

// Master data type variable
tel_net data;
m_omega mst;
tel_state state;
mst2net net_mst;
mst2net net_mst_delay;
slv2net net_slv;
slv2net net_slv_delay;
mst_PO m_PO;
slv_PO s_PO;

// loop state check
bool loop_state = false;

// Slave data type variabl
SLAVEPACK slvServoData; 
DeviceData gServoDeviceData;


extern bool Calibration_state;


HDCallbackCode HDCALLBACK setDeviceForce(void *pUserData)
{
	hdBeginFrame(hdGetCurrentDevice());
	if(Calibration_state)
	{	
		
		//Slave(Phantom) Device Frame Start.
		

		// Master(Omega) Active.
		master_omega_gen(&net_slv_delay, &net_mst, &mst, &m_PO,&state, loop_state);
		

		// generate Time Delay and Packet loss 
		bi_teleoperation_data(&net_mst, &net_mst_delay, &net_slv, &net_slv_delay, &state);
		
		// Slave(Phantom) Active.
		slave_phantom_gen(&net_slv, &net_mst_delay, &slvServoData, &s_PO, &state);



		
	}
	/* Also check the error state of HDAPI. */
	gServoDeviceData.m_error = hdGetError();

	hdEndFrame(hdGetCurrentDevice());
	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error,
			"Error detected while haptic force rendering\n");
		if (hduIsSchedulerError(&error))
		{
			// system state 
			state.system_state = SYSTEM_OFFLINE;
			return HD_CALLBACK_DONE;
		}
	}
	return HD_CALLBACK_CONTINUE;
}


int main()
{
	// FIlE Input Output --- Data logging
	FILE *m_log_fileX = fopen("../mst_po_data_x.txt","wb");
	FILE *m_log_fileY = fopen("../mst_po_data_y.txt","wb");
	FILE *m_log_fileZ = fopen("../mst_po_data_z.txt","wb");

	FILE *s_log_fileX = fopen("../slv_po_data_x.txt","wb");
	FILE *s_log_fileY = fopen("../slv_po_data_y.txt","wb");
	FILE *s_log_fileZ = fopen("../slv_po_data_z.txt","wb");

	FILE *mst_log_fileX = fopen("../mst_data_x.txt","wb");
	FILE *mst_log_fileY = fopen("../mst_data_y.txt","wb");
	FILE *mst_log_fileZ = fopen("../mst_data_z.txt","wb");

	FILE *slv_log_fileX = fopen("../slv_data_x.txt","wb");
	FILE *slv_log_fileY = fopen("../slv_data_y.txt","wb");
	FILE *slv_log_fileZ = fopen("../slv_data_z.txt","wb");

	// Slave device
	HHD hHD;

	int count = 0;
	double Dt = 0.001;
	
	
	mst_PO *m_data = new mst_PO[n];
	slv_PO *s_data = new slv_PO[n];
	m_omega *m_omega_data = new m_omega[n];
	SLAVEPACK *s_phantom_data = new SLAVEPACK[n];

	/*
	// shared memory parameter
	BOOL bEnd = FALSE;
	HANDLE hMapRead;
	
	if(sharedmemory_init())
	{
		printf("shared memory initialize failed! \n");
		return -1;
	}
	*/
	
	// Variable Initailize.
	sharedmemory_init(data,mst,state,net_mst,net_mst_delay,net_slv,net_slv_delay,m_PO,s_PO,slvServoData,gServoDeviceData);
	printf("Omega (Master) to Phantom (Slave) Teleoperation System Variable Intialize complete\n\n\n");


	// Master(Omega) and Slave(Phantom) Initialize.
	// if state callback Calibration by slave phantom.
	master_initialize(&state);
	printf("Master Device (Omega 3) Intialize complete\n\n\n");

	slave_initialize(hHD,&state);
	printf("Slave Device (Phantom Premium) Intialize complete\n\n\n");

	// Schedule the frictionless plane callback, which will then run at 
    // servoloop rates and command forces if the user penetrates the plane.
    HDCallbackCode hTeleopCallback = hdScheduleAsynchronous(
        setDeviceForce, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

	while(!loop_state)
	{
		
		print_state();

		//printf("Teloperation System Control Loop start\n\n\n");
		static clock_t old_t = clock();
		Dt = double(clock() - old_t) / 1000.0;
		old_t = clock();

		// Slave Callback Start point -----
		if (!hdWaitForCompletion(hTeleopCallback, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            getch();
			break;
		}

		/*
		// Shared memory communication ----
		hMapRead = OpenFileMapping(FILE_MAP_ALL_ACCESS,FALSE,"Mst_TeleoperationData");
		data = (tel_net *)MapViewOfFile(hMapRead,FILE_MAP_ALL_ACCESS,0,0,sizeof(tel_net));

		if(data == NULL)
		{
			printf("shared memory read fail\n");
			CloseHandle(hMapRead);
			return -1;
		}

		memcpy(tel_state,data->state,sizeof(tel_state));
		memcpy(net_mst,data->net,sizeof(mst2net));
		*/
		if(Dt == 0.0) continue;
		
		// Data saver
		if(state.m_save == true){
			memcpy(&m_data[count],&m_PO,sizeof(mst_PO));
			memcpy(&s_data[count],&s_PO,sizeof(slv_PO));
			memcpy(&m_omega_data[count],&mst,sizeof(m_omega));
			memcpy(&s_phantom_data[count],&slvServoData,sizeof(SLAVEPACK));
			count++;
		}
		
		
		if(dhdKbHit()) {
			switch(dhdKbGet()){
				case 'q': loop_state = true; break;
			}
		}
		
		
		// detect system state 
		switch(state.system_state){
			case SYSTEM_OFFLINE : loop_state = true; break;
			case SYSTEM_LOADING : break;
			case SYSTEM_ONLINE : break;
		}
		/*
		printf("\n\n");
		printf("%d\n", ft_readbuf->m_head);
		printf("%lf\n", ft_readbuf->sFTforce[0]);
		printf("%lf\n", ft_readbuf->sFTforce[1]);
		printf("%lf\n", ft_readbuf->sFTforce[2]);
		printf("%lf\n", ft_readbuf->sFTtorque[0]);
		printf("%lf\n", ft_readbuf->sFTtorque[1]);
		printf("%lf\n", ft_readbuf->sFTtorque[2]);
		printf("%d\n", ft_readbuf->m_tail);
		printf("\n\n");
		*/
		/*
		printf("\n\n");
		printf("%lf\n", ft_sensor->force[0]);
		printf("%lf\n", ft_sensor->force[1]);
		printf("%lf\n", ft_sensor->force[2]);
		printf("%lf\n", ft_sensor->torque[0]);
		printf("%lf\n", ft_sensor->torque[1]);
		printf("%lf\n", ft_sensor->torque[2]);
		printf("\n\n");
		*/
	}
	
	// Data saver
	if(state.m_save == true){
		for(int i=0;i<count;i++){
			
			// master's Passivity Observer data save area
			fprintf(m_log_fileX, "%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n", 
				m_data[i].m_Data_Em_in[0], m_data[i].m_Data_Em_out_origin[0], m_data[i].m_Data_Em_out[0], m_data[i].m_Data_Es_in[0],
				m_data[i].m_Data_Epc[0], m_data[i].m_Data_Fs_origin[0], m_data[i].m_Data_Fs[0], m_data[i].m_Data_Fvms[0], m_data[i].m_Data_Vm[0], m_data[i].m_Dt);
			fprintf(m_log_fileY, "%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n", 
				m_data[i].m_Data_Em_in[1], m_data[i].m_Data_Em_out_origin[1], m_data[i].m_Data_Em_out[1], m_data[i].m_Data_Es_in[1], 
				m_data[i].m_Data_Epc[1],m_data[i].m_Data_Fs_origin[1], m_data[i].m_Data_Fs[1], m_data[i].m_Data_Fvms[1], m_data[i].m_Data_Vm[1], m_data[i].m_Dt);
			fprintf(m_log_fileZ, "%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n", 
				m_data[i].m_Data_Em_in[2], m_data[i].m_Data_Em_out_origin[2], m_data[i].m_Data_Em_out[2], m_data[i].m_Data_Es_in[2], 
				m_data[i].m_Data_Epc[2],m_data[i].m_Data_Fs_origin[2], m_data[i].m_Data_Fs[2], m_data[i].m_Data_Fvms[2], m_data[i].m_Data_Vm[2], m_data[i].m_Dt);
				
			// slave's Passivity Observer data save area
			fprintf(s_log_fileX, "%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n", 
				s_data[i].s_Data_Es_in[0], s_data[i].s_Data_Es_out_origin[0], s_data[i].s_Data_Es_out[0], s_data[i].s_Data_Em_in[0], 
				s_data[i].s_Data_Epc[0], s_data[i].s_Data_Fs[0], s_data[i].s_Data_Vm_origin[0], s_data[i].s_Data_Vm[0], s_data[i].s_Dt);
			fprintf(s_log_fileY, "%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n", 
				s_data[i].s_Data_Es_in[1], s_data[i].s_Data_Es_out_origin[1], s_data[i].s_Data_Es_out[1], s_data[i].s_Data_Em_in[1], 
				s_data[i].s_Data_Epc[1],s_data[i].s_Data_Fs[1], s_data[i].s_Data_Vm_origin[1], s_data[i].s_Data_Vm[1], s_data[i].s_Dt);
			fprintf(s_log_fileZ, "%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n", 
				s_data[i].s_Data_Es_in[2], s_data[i].s_Data_Es_out_origin[2], s_data[i].s_Data_Es_out[2], s_data[i].s_Data_Em_in[2], 
				s_data[i].s_Data_Epc[2],s_data[i].s_Data_Fs[2], s_data[i].s_Data_Vm_origin[2], s_data[i].s_Data_Vm[2], s_data[i].s_Dt);
			
			/*
			bool m_indexing;
			double m_forcefeedback[DOF];
			double m_pose[DOF];
			double m_prevpose[DOF];
			double m_vel[DOF];
			double m_energy[DOF];
			double m_dt;
			*/
			fprintf(mst_log_fileX, "%6.6f %6.6f %6.6f %6.6f %6.6f\n",m_omega_data[i].m_forcefeedback[0],m_omega_data[i].m_pose[0],
				m_omega_data[i].m_prevpose[0],m_omega_data[i].m_vel[0],m_omega_data[i].m_energy[0]);
			fprintf(mst_log_fileY, "%6.6f %6.6f %6.6f %6.6f %6.6f\n",m_omega_data[i].m_forcefeedback[1],m_omega_data[i].m_pose[1],
				m_omega_data[i].m_prevpose[1],m_omega_data[i].m_vel[1],m_omega_data[i].m_energy[1]);
			fprintf(mst_log_fileZ, "%6.6f %6.6f %6.6f %6.6f %6.6f\n",m_omega_data[i].m_forcefeedback[2],m_omega_data[i].m_pose[2],
				m_omega_data[i].m_prevpose[2],m_omega_data[i].m_vel[2],m_omega_data[i].m_energy[2]);

			/*
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
			*/
			fprintf(slv_log_fileX,"%6.6f %6.6f %6.6f %6.6f %6.6f\n",s_phantom_data[i].position[0],s_phantom_data[i].prev[0],
				s_phantom_data[i].force[0],s_phantom_data[i].disp[0],s_phantom_data[i].s_vel[0]);
			fprintf(slv_log_fileY,"%6.6f %6.6f %6.6f %6.6f %6.6f\n",s_phantom_data[i].position[1],s_phantom_data[i].prev[1],
				s_phantom_data[i].force[1],s_phantom_data[i].disp[1],s_phantom_data[i].s_vel[1]);
			fprintf(slv_log_fileZ,"%6.6f %6.6f %6.6f %6.6f %6.6f\n",s_phantom_data[i].position[2],s_phantom_data[i].prev[2],
				s_phantom_data[i].force[2],s_phantom_data[i].disp[2],s_phantom_data[i].s_vel[2]);

		}
	}

	// Master Omega --- Close the connection
	dhdClose();
	drdClose();
	
	// Stop Scheduler
	hdStopScheduler();
	hdUnschedule(hTeleopCallback);

	// Slave Phantom -- Close the connection 
	hdDisableDevice(hHD);

	// Shared Memory Release
	//UnmapViewOfFile(data);
	//CloseHandle(hMapRead);

	// FILE I/O close connection
	fclose(m_log_fileX);
	fclose(m_log_fileY);
	fclose(m_log_fileZ);
	fclose(s_log_fileX);
	fclose(s_log_fileY);
	fclose(s_log_fileZ);

	fclose(mst_log_fileX);
	fclose(mst_log_fileY);
	fclose(mst_log_fileZ);
	fclose(slv_log_fileX);
	fclose(slv_log_fileY);
	fclose(slv_log_fileZ);
	
	exit_all_program();

	return 0;

}