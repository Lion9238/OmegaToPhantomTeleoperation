#include<stdio.h>
#include<iostream>


class OmegaToPhantomDimension
{
public:
	OmegaToPhantomDimension::OmegaToPhantomDimension();
	OmegaToPhantomDimension::~OmegaToPhantomDimension();
	int OmegaToPhantomDimensionChange(double omega_x, double omega_y, double omega_z,double &output_x, double &output_y, double &output_z);
	int PhantomToOmegaDimensionChange(double phantom_x, double phantom_y, double phantom_z, double &output_x, double &output_y, double &output_z);
	int FTsensorToPhantomDimensionChange(double ft_x, double ft_y, double ft_z, double &phantom_x, double &phantom_y, double &phantom_z);

private:
	int OmegaToPhantomReset();
	int PhantomToOmegaReset();
	int FTsensorToPhantomReset();

private:
	// Omega to Phantom Dimension Change
	double O2P_omega_x;
	double O2P_omega_y;
	double O2P_omega_z;
	double O2P_phantom_x;
	double O2P_phantom_y;
	double O2P_phantom_z;

	// Phantom to Omega Dimension Change
	double P2O_omega_x;
	double P2O_omega_y;
	double P2O_omega_z;
	double P2O_phantom_x;
	double P2O_phantom_y;
	double P2O_phantom_z;

	// FTsensor to Phantom Dimesnion Change
	double F2P_ft_x;
	double F2P_ft_y;
	double F2P_ft_z;
	double F2P_phantom_x;
	double F2P_phantom_y;
	double F2P_phantom_z;
};