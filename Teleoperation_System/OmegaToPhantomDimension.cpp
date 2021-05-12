#include "OmegaToPhantomDimension.h"


OmegaToPhantomDimension::OmegaToPhantomDimension()
{
	// Omega to Phantom Dimension Change
	O2P_omega_x = 0.0;
	O2P_omega_y  = 0.0;
	O2P_omega_z  = 0.0;
	O2P_phantom_x  = 0.0;
	O2P_phantom_y  = 0.0;
	O2P_phantom_z  = 0.0;

	// Phantom to Omega Dimension Change
	P2O_omega_x = 0.0;
	P2O_omega_y = 0.0;
	P2O_omega_z = 0.0;
	P2O_phantom_x = 0.0;
	P2O_phantom_y = 0.0;
	P2O_phantom_z = 0.0;
}

OmegaToPhantomDimension::~OmegaToPhantomDimension()
{
	PhantomToOmegaReset();
	OmegaToPhantomReset();
	FTsensorToPhantomReset();
}
int OmegaToPhantomDimension::OmegaToPhantomReset()
{
	// Omega to Phantom Dimension Change
	O2P_omega_x = 0.0;
	O2P_omega_y  = 0.0;
	O2P_omega_z  = 0.0;
	O2P_phantom_x  = 0.0;
	O2P_phantom_y  = 0.0;
	O2P_phantom_z  = 0.0;

	return 1;
}
int OmegaToPhantomDimension::PhantomToOmegaReset()
{
	// Phantom to Omega Dimension Change
	P2O_omega_x = 0.0;
	P2O_omega_y = 0.0;
	P2O_omega_z = 0.0;
	P2O_phantom_x = 0.0;
	P2O_phantom_y = 0.0;
	P2O_phantom_z = 0.0;

	return 1;
}

int OmegaToPhantomDimension::FTsensorToPhantomReset()
{
	// Phantom to Omega Dimension Change
	F2P_ft_x = 0.0;
	F2P_ft_y = 0.0;
	F2P_ft_z = 0.0;
	F2P_phantom_x = 0.0;
	F2P_phantom_y = 0.0;
	F2P_phantom_z = 0.0;

	return 1;
}



int OmegaToPhantomDimension::OmegaToPhantomDimensionChange(double omega_x, double omega_y, double omega_z, 
	double &output_x, double &output_y, double &output_z)
{
	if(!OmegaToPhantomReset())
		return -1;

	O2P_omega_x = omega_x;
	O2P_omega_y = omega_y;
	O2P_omega_z = omega_z;
	//printf("before : %f %f %f\n",O2P_omega_x,O2P_omega_y,O2P_omega_z);
	O2P_phantom_x  = O2P_omega_y;
	O2P_phantom_y  = O2P_omega_z;
	O2P_phantom_z  = O2P_omega_x;
	//printf("after : %f %f %f\n",O2P_phantom_x,O2P_phantom_y,O2P_phantom_z);
	output_x = O2P_phantom_x;
	output_y = O2P_phantom_y;
	output_z = O2P_phantom_z;

	return 1;
}

int OmegaToPhantomDimension::PhantomToOmegaDimensionChange(double phantom_x, double phantom_y, double phantom_z,
	double &output_x, double &output_y, double &output_z)
{
	if(!PhantomToOmegaReset())
		return -1;

	P2O_phantom_x = phantom_x;
	P2O_phantom_y = phantom_y;
	P2O_phantom_z = phantom_z;

	P2O_omega_x = P2O_phantom_z;
	P2O_omega_y = P2O_phantom_x;
	P2O_omega_z = P2O_phantom_y;

	output_x = P2O_omega_x;
	output_y = P2O_omega_y;
	output_z = P2O_omega_z;

	return 1;

}


int OmegaToPhantomDimension::FTsensorToPhantomDimensionChange(double ft_x, double ft_y, double ft_z,
	double &phantom_x,double &phantom_y,double &phantom_z)
{
	F2P_ft_x = ft_x;
	F2P_ft_y = ft_y;
	F2P_ft_z = ft_z;
	
	F2P_phantom_x = F2P_ft_x;
	F2P_phantom_y = -1 * F2P_ft_z;
	F2P_phantom_z = F2P_ft_y;

	phantom_x = F2P_phantom_x;
	phantom_y = F2P_phantom_y;
	phantom_z = F2P_phantom_z;
	
	return 1;

}
