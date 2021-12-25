/*******************************************************************************
 * The MIT License (MIT)
 * 
 * Copyright (c) 2020-2021 Andrey Palyanov
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the MIT License
 * which accompanies this distribution, and is available at
 * http://opensource.org/licenses/MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *******************************************************************************/

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <iterator>
#include <vector>

using namespace std;

#include "owPhysicsFluidSimulator.h"

int numOfElasticConnections = 0; // Number of elastic connection TODO: move this to owConfig class
int numOfLiquidP = 0;			 // Number of liquid particles TODO: move this to owConfig class
int numOfElasticP = 0;			 // Number of liquid particles TODO: move this to owConfig class
int numOfBoundaryP = 0;			 // Number of boundary particles TODO: move this to owConfig class
int numOfMembranes = 0;			 // Number of membranes TODO: move this to owConfig class
extern float * muscle_activation_signal_cpp;
extern float spike_pos[496];
extern float spike_time[496][50];
int iter_step = 10;				 // Count of iteration which will be skipped before logging configuration to file
								 // NOTE: this using only in "load config to file" mode

std::vector<int> memParticle;
std::vector<int> muscleParticle;
void fillMemId(int * particleMembranesList_cpp){
	for(int i=0;i < numOfElasticP ;i++){
		if(particleMembranesList_cpp[MAX_MEMBRANES_INCLUDING_SAME_PARTICLE * i + 0]!=-1){
			memParticle.push_back(i);
		}
	}
	std::cout << memParticle.size() << std::endl;
}
void fillMuscleParticles(float * elasticConnection){
	for(int i=0;i < numOfElasticP;i++){
		for(int j=0;j<MAX_NEIGHBOR_COUNT;j++)
		{
			if((int)elasticConnection[i * MAX_NEIGHBOR_COUNT * 4 + j * 4 + 2] != 0){
				muscleParticle.push_back(i);
				break;
			}
		}
	}
	if(muscleParticle.size()>0) owHelper::log_buffer(&muscleParticle[0],1,muscleParticle.size(),"./logs/muscleParticles");
	std::cout << memParticle.size() << std::endl;
}
/** Constructor method for owPhysicsFluidSimulator.
 *
 *  @param helper
 *  pointer to owHelper object with helper function.
 *  @param dev_type
 *  defines preferable device type for current configuration
 */
owPhysicsFluidSimulator::owPhysicsFluidSimulator(owHelper * helper,int argc, char ** argv)
{
	//int generateInitialConfiguration = 1;//1 to generate initial configuration, 0 - load from file

	try{
		iterationCount = 0;
		config = new owConfigProperty(argc, argv);
#if generateBodyConfiguration
		config->xmin = 0.f;
		config->xmax = (35+10)*h;//35.f*h;//60//6.f*h;//
		config->ymin = 0.f;
		config->ymax = (25+10)*h;//20.f*h;//29.0f*h;//15.f//30//7.f*h;// set_box_size
		config->zmin = 0.f;
		config->zmax = (125+35+10)*h;//125.f*h;//125.f/*75.f**/*h;//30.f*h;//80.f*h;//160.0f*h;//140//4.f*h;//
#endif
		if(generateBodyConfiguration)
		// GENERATE THE SCENE
		owHelper::generateConfiguration(0, position_cpp, velocity_cpp, elasticConnectionsData_cpp, membraneData_cpp, numOfLiquidP, numOfElasticP, numOfBoundaryP, numOfElasticConnections, numOfMembranes, particleMembranesList_cpp, config);
		else								
		// LOAD FROM FILE
		owHelper::preLoadConfiguration(numOfMembranes, config, numOfLiquidP, numOfElasticP, numOfBoundaryP);

		//TODO move initialization to configuration class
		config->gridCellsX = (int)( ( config->xmax - config->xmin ) / h ) + 1;
		config->gridCellsY = (int)( ( config->ymax - config->ymin ) / h ) + 1;
		config->gridCellsZ = (int)( ( config->zmax - config->zmin ) / h ) + 1;
		config->gridCellCount = config->gridCellsX * config->gridCellsY * config->gridCellsZ;
		//
		position_cpp = new float[ 4 * config->getParticleCount() ];
		velocity_cpp = new float[ 4 * config->getParticleCount() ];
		muscle_activation_signal_cpp = new float [MUSCLE_COUNT];
		if(numOfElasticP != 0)
			elasticConnectionsData_cpp = new float[ 4 * numOfElasticP * MAX_NEIGHBOR_COUNT ];
		if(numOfMembranes<=0)
			membraneData_cpp = NULL;
		else
			membraneData_cpp = new int [numOfMembranes*3];
		if(numOfElasticP<=0)
			particleMembranesList_cpp = NULL;
		else
			particleMembranesList_cpp = new int [numOfElasticP*MAX_MEMBRANES_INCLUDING_SAME_PARTICLE];
		for(int i=0;i<MUSCLE_COUNT;i++)
		{
			muscle_activation_signal_cpp[i] = 0.f;
		}

	std::ifstream tadpole_mnrn_activity_file ("muscle_activity_data/spk_mns_data-49.m", std::ios_base::binary);

	float min_time = 1000;

	if( tadpole_mnrn_activity_file.is_open() )
	{
		string mnrn_s;
		float mnrn_f;
		char mnrn_c;
		int mnrn_n,tn;
		int result;
		

		while( tadpole_mnrn_activity_file.good() )
		{
			tadpole_mnrn_activity_file >> mnrn_s;

			if(mnrn_s=="pos_mns")
			{
				mnrn_n = 0;
				tadpole_mnrn_activity_file >> mnrn_s;
				tadpole_mnrn_activity_file >> mnrn_c;

				while( tadpole_mnrn_activity_file.good() )
				{
					tadpole_mnrn_activity_file >> mnrn_s;
					mnrn_f = (float)atof(mnrn_s.c_str());
					if(mnrn_f>0)
					{
						spike_pos[mnrn_n] = mnrn_f;
						mnrn_n++;
					}
					if(mnrn_n==496) 
					{
						mnrn_n = 0;
						break;
					}
				}

			}


			if( (result = mnrn_s.find("spk_mns{"))>=0 ) 
			{
				tn = 0;

				tadpole_mnrn_activity_file >> mnrn_s;
				tadpole_mnrn_activity_file >> mnrn_c;

				while( tadpole_mnrn_activity_file.good() )
				{
					tadpole_mnrn_activity_file >> mnrn_s;
					//mnrn_f = atof(mnrn_s.c_str())-200;// time shift // older value
					//mnrn_f = atof(mnrn_s.c_str())-105;// time shift for data-83
					mnrn_f = (float)atof(mnrn_s.c_str());// no time shift
					if(mnrn_f>0)
					{
						spike_time[mnrn_n][tn] = mnrn_f;
						tn++;
						min_time = min(min_time,mnrn_f);
					}
					if((result=mnrn_s.find("];"))>=0) 
					{
						spike_time[mnrn_n][tn] = -1.f;
						break;
					}
				}

				mnrn_n++;
			}
		}

		tadpole_mnrn_activity_file.close();
	}

	//make left and right signal identical
	/*
	for(int mnrn_n=0;mnrn_n<248;mnrn_n++)
	{
		for(int tn=0;tn<50;tn++)
		{
			spike_time[mnrn_n][tn] = spike_time[mnrn_n+248][tn] + 24.f*(spike_time[mnrn_n+248][tn]>0);
		}
	}
	*/
	

	// upper - direct simulation signals
	//============================================
	// below - curves approximated with polynomes
/*
	int j;
	float ppl_coeffs [19][4] = {
		{ -4.02E-10,	4.78E-06,	-0.009343848,	300.3267975 },
		{ -3.45E-10,	4.51E-06,	-0.009099102,	353.8947042 },
		{ -1.63E-09,	9.94E-06,	-0.016340320,	411.5204132 },
		{ -1.55E-09,	9.64E-06,	-0.016021312,	466.7398642 },
		{ -1.53E-09,	9.67E-06,	-0.016269002,	522.7270883 },
		{ -1.37E-09,	8.95E-06,	-0.015279049,	578.4383276 },
		{ -1.46E-09,	9.44E-06,	-0.016110819,	635.2823577 },
		{ -1.35E-09,	8.91E-06,	-0.015271069,	691.4122555 },
		{ -1.27E-09,	8.62E-06,	-0.014975325,	748.0174550 },
		{ -1.31E-09,	8.76E-06,	-0.015094891,	804.7739523 },
		{ -1.33E-09,	8.87E-06,	-0.015288872,	861.6931115 },
		{ -1.36E-09,	8.96E-06,	-0.015363505,	918.5155817 },
		{ -1.20E-09,	8.34E-06,	-0.014610728,	975.1152075 },
		{ -1.19E-09,	8.25E-06,	-0.014430192,	1031.874861 },
		{ -1.20E-09,	8.33E-06,	-0.014616299,	1088.891825 },
		{ -1.17E-09,	8.20E-06,	-0.014378895,	1145.640768 },
		{ -1.18E-09,	8.25E-06,	-0.014511168,	1202.641283 },
		{ -1.13E-09,	8.01E-06,	-0.014132960,	1259.337354 },
		{ -1.17E-09,	8.22E-06,	-0.014454351,	1316.403945 } };

	float ppr_coeffs [20][4] = {
		{ 1.95E-09,	-4.00E-06,	0.001154413,	268.8034822 },
		{ 2.05E-09,	-4.57E-06,	0.001995978,	321.5528985 },
		{ 2.08E-09,	-4.80E-06,	0.002342423,	375.6151045 },
		{ 2.09E-09,	-5.08E-06,	0.002762468,	430.5190153 },
		{ 2.36E-09,	-6.20E-06,	0.004132688,	485.5691872 },
		{ 2.28E-09,	-5.93E-06,	0.003832186,	541.6692037 },
		{ 2.38E-09,	-6.32E-06,	0.004291043,	597.7821365 },
		{ 2.26E-09,	-5.80E-06,	0.003610606,	654.5593126 },
		{ 2.45E-09,	-6.62E-06,	0.004737834,	710.6732448 },
		{ 2.26E-09,	-5.84E-06,	0.003669362,	767.8652697 },
		{ 2.43E-09,	-6.51E-06,	0.004530952,	824.2894051 },
		{ 2.28E-09,	-5.89E-06,	0.003732581,	881.4442814 },
		{ 2.42E-09,	-6.48E-06,	0.004500539,	937.9588130 },
		{ 2.28E-09,	-5.92E-06,	0.003817930,	995.0986192 },
		{ 2.44E-09,	-6.56E-06,	0.004611772,	1051.672106 },
		{ 2.24E-09,	-5.75E-06,	0.003542127,	1109.013811 },
		{ 2.42E-09,	-6.47E-06,	0.004502088,	1165.497092 },
		{ 2.25E-09,	-5.77E-06,	0.003555866,	1222.809819 },
		{ 2.45E-09,	-6.63E-06,	0.004777700,	1279.144140 },
		{ 2.27E-09,	-5.87E-06,	0.003707960,	1336.536195 } };


	int mnrn_n, tn;
	float t;

	for(mnrn_n=0;mnrn_n<248;mnrn_n++)
	{
		t = 500.f + 2000.f*mnrn_n/247.f;
		spike_pos[mnrn_n    ] = t;
		spike_pos[mnrn_n+248] = t;

		for(tn=0;tn<19;tn++)
		{
			spike_time[mnrn_n][tn] = -200+t*t*t*ppl_coeffs[tn][0] + t*t*ppl_coeffs[tn][1] + t*ppl_coeffs[tn][2] + ppl_coeffs[tn][3];
		}
		spike_time[mnrn_n][19] = -1;

		for(tn=0;tn<20;tn++)
		{
			spike_time[mnrn_n+248][tn] = -200+t*t*t*ppr_coeffs[tn][0] + t*t*ppr_coeffs[tn][1] + t*ppr_coeffs[tn][2] + ppr_coeffs[tn][3];
		}
		spike_time[mnrn_n+248][20] = -1;
	}
	*/
		//================================

		//The buffers listed below are only for usability and debug
		density_cpp = new float[ 1 * config->getParticleCount() ];
		particleIndex_cpp = new unsigned int[config->getParticleCount() * 2];
		if(generateBodyConfiguration)
			// GENERATE THE SCENE
			owHelper::generateConfiguration(1,position_cpp, velocity_cpp, elasticConnectionsData_cpp, membraneData_cpp, numOfLiquidP, numOfElasticP, numOfBoundaryP, numOfElasticConnections, numOfMembranes, particleMembranesList_cpp, config );
		else 
			// LOAD FROM FILE
			owHelper::loadConfiguration( position_cpp, velocity_cpp, elasticConnectionsData_cpp, numOfLiquidP, numOfElasticP, numOfBoundaryP, numOfElasticConnections, numOfMembranes,membraneData_cpp, particleMembranesList_cpp, config );		//Load configuration from file to buffer
		fillMuscleParticles(elasticConnectionsData_cpp);
		if(numOfElasticP != 0){
			ocl_solver = new owOpenCLSolver(position_cpp, velocity_cpp, config, elasticConnectionsData_cpp, membraneData_cpp, particleMembranesList_cpp);	//Create new openCLsolver instance
		}else
			ocl_solver = new owOpenCLSolver(position_cpp,velocity_cpp, config);	//Create new openCLsolver instance
		this->helper = helper;
	}catch( std::exception &e ){
		std::cout << "ERROR_08: " << e.what() << std::endl;
		exit( -8 );
	}
}
/** Reset simulation
 *
 *  Restart simulation with new or current simulation configuration.
 *  It redefines all required data buffers and restart owOpenCLSolver
 *  by run owOpenCLSolver::reset(...).
 */
void owPhysicsFluidSimulator::reset(){
	iterationCount = 0;
	numOfBoundaryP = 0;
	numOfElasticP = 0;
	numOfLiquidP = 0;
	numOfMembranes = 0;
	numOfElasticConnections = 0;
#if generateBodyConfiguration
		config->xmin = 0.f;
		config->xmax = 30.0f*h;
		config->ymin = 0.f;
		config->ymax = 20.0f*h;
		config->zmin = 0.f;
		config->zmax = 60.0f*h;//200
#endif
	if(generateBodyConfiguration)
	// GENERATE THE SCENE
	owHelper::generateConfiguration(0, position_cpp, velocity_cpp, elasticConnectionsData_cpp, membraneData_cpp, numOfLiquidP, numOfElasticP, numOfBoundaryP, numOfElasticConnections, numOfMembranes, particleMembranesList_cpp, config);
	else
	// LOAD FROM FILE
	owHelper::preLoadConfiguration(numOfMembranes, config, numOfLiquidP, numOfElasticP, numOfBoundaryP);

	//TODO move initialization to configuration class
	config->gridCellsX = (int)( ( config->xmax - config->xmin ) / h ) + 1;
	config->gridCellsY = (int)( ( config->ymax - config->ymin ) / h ) + 1;
	config->gridCellsZ = (int)( ( config->zmax - config->zmin ) / h ) + 1;
	config->gridCellCount = config->gridCellsX * config->gridCellsY * config->gridCellsZ;
	//
	position_cpp = new float[ 4 * config->getParticleCount() ];
	velocity_cpp = new float[ 4 * config->getParticleCount() ];

	muscle_activation_signal_cpp = new float [MUSCLE_COUNT];
	if(numOfElasticP != 0) elasticConnectionsData_cpp = new float[ 4 * numOfElasticP * MAX_NEIGHBOR_COUNT ];
	if(numOfMembranes<=0) membraneData_cpp = NULL; else membraneData_cpp = new int [ numOfMembranes * 3 ];
	if(numOfElasticP<=0)  particleMembranesList_cpp = NULL; else particleMembranesList_cpp = new int [numOfElasticP*MAX_MEMBRANES_INCLUDING_SAME_PARTICLE];
	for(int i=0;i<MUSCLE_COUNT;i++)
	{
		muscle_activation_signal_cpp[i] = 0.f;
	}

	//The buffers listed below are only for usability and debug
	density_cpp = new float[ 1 * config->getParticleCount() ];
	particleIndex_cpp = new unsigned int[config->getParticleCount() * 2];

	if(generateBodyConfiguration)
	// GENERATE THE SCENE
	owHelper::generateConfiguration(1,position_cpp, velocity_cpp, elasticConnectionsData_cpp, membraneData_cpp, numOfLiquidP, numOfElasticP, numOfBoundaryP, numOfElasticConnections, numOfMembranes, particleMembranesList_cpp, config );
	else
	// LOAD FROM FILE
	owHelper::loadConfiguration( position_cpp, velocity_cpp, elasticConnectionsData_cpp, numOfLiquidP, numOfElasticP, numOfBoundaryP, numOfElasticConnections, numOfMembranes,membraneData_cpp, particleMembranesList_cpp, config );		//Load configuration from file to buffer
	if(numOfElasticP != 0){
		ocl_solver->reset(position_cpp, velocity_cpp, config, elasticConnectionsData_cpp, membraneData_cpp, particleMembranesList_cpp);	//Create new openCLsolver instance
	}else
		ocl_solver->reset(position_cpp,velocity_cpp, config);	//Create new openCLsolver instance
}
/** Run one simulation step
 *
 *  Run simulation step in pipeline manner.
 *  It starts with neighbor search algorithm than
 *  physic simulation algorithms: PCI SPH [1],
 *  elastic matter simulation, boundary handling [2],
 *  membranes handling and finally numerical integration.
 *  [1] http://www.ifi.uzh.ch/vmml/publications/pcisph/pcisph.pdf
 *  [2] M. Ihmsen, N. Akinci, M. Gissler, M. Teschner,
 *      Boundary Handling and Adaptive Time-stepping for PCISPH
 *      Proc. VRIPHYS, Copenhagen, Denmark, pp. 79-88, Nov 11-12, 2010
 *
 *  @param looad_to
 *  If it's true than Sibernetic works "load simulation data in file" mode.
 */
double owPhysicsFluidSimulator::simulationStep(const bool load_to)
{
	int iter = 0;//PCISPH prediction-correction iterations counter

	//if(iterationCount==0) return 0.0;//uncomment this line to stop movement of the scene
	//==============================
	// 2020 optimize positions of particles composing tadpole vertical midplane
	//==============================
	
	float r0 = 2.0f*1.5865f;
	float r0_2 = r0*r0;
	float r0_6 = r0_2*r0_2*r0_2;
	float r0_12 = r0_6*r0_6;

 	//return helper->get_elapsedTime();

	//==============================
	//iterationCount++;
	
	helper->refreshTime();
	printf("\n[[ Step %d ]]\n",iterationCount);

	//return helper->get_elapsedTime();

	try{
		//SEARCH FOR NEIGHBOURS PART
		//ocl_solver->_runClearBuffers();								helper->watch_report("_runClearBuffers: \t%9.3f ms\n");
		int i = 0;
		ocl_solver->_runHashParticles(config);							helper->watch_report("_runHashParticles: \t%9.3f ms\n");
		/*ocl_solver->read_position_buffer(position_cpp, config);
		for (i = 0; i < config->getParticleCount(); i++)
		{
			//if (!(((int)position_cpp[4 * i + 3] * 1000) == 2477))
			{
				printf("%d: %f\n", i, position_cpp[4 * i + 3]);
			}
		}*/
		//exit(1);
		ocl_solver->_runSort(config);									helper->watch_report("_runSort: \t\t%9.3f ms\n");
		ocl_solver->_runSortPostPass(config);							helper->watch_report("_runSortPostPass: \t%9.3f ms\n");
		ocl_solver->_runIndexx(config);									helper->watch_report("_runIndexx: \t\t%9.3f ms\n");
		ocl_solver->_runIndexPostPass(config);							helper->watch_report("_runIndexPostPass: \t%9.3f ms\n");
		ocl_solver->_runFindNeighbors(config);							helper->watch_report("_runFindNeighbors: \t%9.3f ms\n");
		//PCISPH PART
		if(config->getIntegrationMethod() == LEAPFROG){ // in this case we should remember value of position on stem i - 1
			//Calc next time (t+dt) positions x(t+dt)
			ocl_solver->_run_pcisph_integrate(iterationCount,0/*=positions_mode*/, config);
		}
		ocl_solver->_run_pcisph_computeDensity(config);
		ocl_solver->_run_pcisph_computeForcesAndInitPressure(iterationCount,config);
		ocl_solver->_run_pcisph_computeElasticForces(config);

		int d_min = 10000, d_max = 0;
		float rho_more_rho0 = 0;
		float rho_less_rho0 = 0;
		int cnt_rmr0 = 0;
		int cnt_rlr0 = 0;
		do{
			//printf("\n^^^^ iter %d ^^^^\n",iter);
			ocl_solver->_run_pcisph_predictPositions(config);
			ocl_solver->_run_pcisph_predictDensity(config);
			float* d_cpp = getDensity_cpp();
			
			for(int id = 0; id < config->getParticleCount(); id++)
			{
				if(d_cpp[id]>=rho0)
				{
					rho_more_rho0 += d_cpp[id];
					cnt_rmr0++;
				}
				else
				{
					rho_less_rho0 += d_cpp[id];
					cnt_rlr0++;
				}
			}

			printf("[iter: %d]", iter);
			if(cnt_rmr0>0) printf("%d (%d), ", (int)(cnt_rmr0/cnt_rmr0), cnt_rmr0);
			if(cnt_rlr0>0) printf("%d (%d)", (int)(cnt_rlr0/cnt_rlr0), cnt_rlr0);
			printf("\n");

			ocl_solver->_run_pcisph_correctPressure(config);
			ocl_solver->_run_pcisph_computePressureForceAcceleration(iterationCount, config);
			iter++;
		}while( iter < maxIteration );

		//and finally calculate v(t+dt)
		if(config->getIntegrationMethod() == LEAPFROG){
			ocl_solver->_run_pcisph_integrate(iterationCount,1/*=velocities_mode*/, config);		helper->watch_report("_runPCISPH: \t\t%9.3f ms\t3 iteration(s)\n");
		}
		else
		{
			ocl_solver->_run_pcisph_integrate(iterationCount, 2,config);		helper->watch_report("_runPCISPH: \t\t%9.3f ms\t3 iteration(s)\n");
		}
		//Handling of Interaction with membranes
		//!!! 03.2021 switched off since no membranes are in use
		/*
		if(numOfMembranes > 0){
			ocl_solver->_run_clearMembraneBuffers(config);
			ocl_solver->_run_computeInteractionWithMembranes(config);
			// compute change of coordinates due to interactions with membranes
			ocl_solver->_run_computeInteractionWithMembranes_finalize(config);
																		helper->watch_report("membraneHandling: \t%9.3f ms\n");
		}*/
		//END
		ocl_solver->read_position_buffer(position_cpp, config);				helper->watch_report("_readBuffer: \t\t%9.3f ms\n");

		/*
		if((iterationCount>0)&&(iterationCount%2==0)&&(iterationCount<12500)) // %100
		{
			int i_pos = 0;
			float *positionVector;
			int done = 0;

			for(int i_pos = 0; i_pos < config->getParticleCount(); i_pos++)
			{
				positionVector = position_cpp + 4 * (+i_pos);
				if(positionVector[ 0 ] < config->xmax/10)
				{
					positionVector[ 0 ] = config->xmax*(0.505f)+r0*(-50+rand()%100)/100.f;
					positionVector[ 1 ] = config->ymax*(0.370f)+r0*(-50+rand()%100)/100.f;//vertical axis
					positionVector[ 2 ] = config->zmax*(0.300f)+r0*(-50+rand()%100)/100.f; 
					positionVector[ 3 ] = 2.4f;// elastic matter, white?
					done = 1;
				}

				if(done) break;
			}

			printf("\n[[ %d - modify leftmost particle position]]\n",iterationCount);
			ocl_solver->copy_buffer_to_device2(position_cpp, config->getParticleCount() * sizeof( float ) * 4);
		}/**/


		//END PCISPH algorithm
		//printf("------------------------------------\n");
		//printf("_Total_step_time:\t%9.3f ms\n",helper->get_elapsedTime());
		//printf("------------------------------------\n");
		if(load_to){
			if(iterationCount == 0){
				owHelper::loadConfigurationToFile(position_cpp, config, muscleParticle, elasticConnectionsData_cpp,membraneData_cpp,true);
			}else{
				if(iterationCount % iter_step == 0){
					owHelper::loadConfigurationToFile(position_cpp, config, muscleParticle, NULL, NULL, false);
				}
			}
		}
		iterationCount++;

		int sp,st;
		int ticks_since_prev_spike=-1;
		int ticks_since_curr_spike=-1;
		float prev_spike_time, curr_spike_time,curr_time;
		//float curr_L_muscle_activity[23];
		//float curr_R_muscle_activity[23];
		float single_mnrn_activity;
		float rest_activity;
		//int mi;
		int m_i=-2;

		float m_a_s[25][9] = {  { 2.00f, 1.5f, 0.7f,-1.00f,-2.00f,-1.5f,-0.7f, 1.00f, 2.00f},// 0
								{ 2.00f, 1.5f, 0.7f,-1.00f,-2.00f,-1.5f,-0.7f, 1.00f, 2.00f},// 1
								{ 2.00f, 1.5f, 0.7f,-0.80f,-2.00f,-1.5f,-0.7f, 0.80f, 2.00f},// 2
								{ 2.00f, 1.5f, 0.7f,-0.60f,-2.00f,-1.5f,-0.7f, 0.60f, 2.00f},// 3
								{ 2.00f, 1.5f, 0.7f,-0.30f,-2.00f,-1.5f,-0.7f, 0.30f, 2.00f},// 4
								{ 2.00f, 1.5f, 0.7f, 0.00f,-2.00f,-1.5f,-0.7f, 0.00f, 2.00f},// 5
								{ 1.90f, 1.5f, 0.7f, 0.25f,-1.90f,-1.5f,-0.7f,-0.25f, 1.90f},// 6
								{ 1.80f, 1.5f, 0.8f, 0.30f,-1.80f,-1.5f,-0.8f,-0.30f, 1.80f},// 7
								{ 1.70f, 1.5f, 0.9f, 0.40f,-1.70f,-1.5f,-0.9f,-0.40f, 1.70f},// 8
								{ 1.60f, 1.5f, 0.9f, 0.41f,-1.60f,-1.5f,-0.9f,-0.41f, 1.60f},// 9
								{ 1.60f, 1.5f, 0.9f, 0.41f,-1.60f,-1.5f,-0.9f,-0.41f, 1.60f},//10
								{ 1.40f, 1.0f, 0.8f, 0.41f,-1.40f,-1.0f,-0.8f,-0.41f, 1.40f},//11
								{ 1.20f, 0.5f, 0.7f, 0.41f,-1.20f,-0.5f,-0.7f,-0.41f, 1.20f},//12
								{ 1.00f, 0.0f, 0.6f, 0.41f,-1.00f, 0.0f,-0.6f,-0.41f, 1.00f},//13
								{ 0.50f,-0.5f, 0.6f, 0.41f,-0.50f, 0.5f,-0.6f,-0.41f, 0.50f},//14
								{ 0.00f,-0.5f, 0.5f, 0.41f, 0.00f, 0.5f,-0.5f,-0.41f, 0.00f},//15
								{ 0.00f,-0.5f, 0.5f, 0.41f, 0.00f, 0.5f,-0.5f,-0.41f, 0.00f},//16
								{ 0.00f,-0.5f, 0.4f, 0.41f, 0.00f, 0.5f,-0.4f,-0.41f, 0.00f},//17
								{-0.15f,-0.5f, 0.4f, 0.41f, 0.15f, 0.5f,-0.4f,-0.41f,-0.15f},//18
								{-0.30f,-0.5f, 0.3f, 0.41f, 0.30f, 0.5f,-0.3f,-0.41f,-0.30f},//19
								{-0.40f,-0.5f, 0.3f, 0.41f, 0.40f, 0.5f,-0.3f,-0.41f,-0.40f},//20
								{-0.40f,-0.5f, 0.2f, 0.41f, 0.40f, 0.5f,-0.2f,-0.41f,-0.40f},//21
								{-0.40f,-0.5f, 0.2f, 0.41f, 0.40f, 0.5f,-0.2f,-0.41f,-0.40f},//22
								{-0.30f,-0.5f, 0.1f, 0.41f, 0.30f, 0.5f,-0.1f,-0.41f,-0.30f},//23
								{-0.20f,-0.5f, 0.1f, 0.41f, 0.20f, 0.5f,-0.1f,-0.41f,-0.20f} //24
							};

		int time_point = (iterationCount%10000)/1250;//0...8
		int t_interval = (iterationCount%1250);

		for(m_i=0;m_i<45;m_i++)
		{
			muscle_activation_signal_cpp[	m_i] = 0.f;
			muscle_activation_signal_cpp[50+m_i] = 0.f;
		}

		for(sp=0;sp<496;sp++)
		{
			single_mnrn_activity = 0;
			curr_time=(float)(iterationCount)*config->getTimeStep()*1000.f + 1*25.f - 50.f + 0*100.f;//+0*100.f;//ms //time shift
			prev_spike_time = curr_spike_time = -1;

			for(st=0;st<50;st++)
			{
				if(spike_time[sp][st]>=0) 
				{
					if(spike_time[sp][st]<curr_time)
					{
						prev_spike_time = curr_spike_time;
						curr_spike_time = spike_time[sp][st]; 
						curr_spike_time += 0.f;
					}
					else
					{
						break;
					}
				}
				else
				{
					break;
				}
			}

			if( (curr_time - curr_spike_time < 60.f)&&(curr_spike_time > 0.f) )
			{
				if( (curr_time - prev_spike_time < 120.f)&&(prev_spike_time > 0.f) )
				{
					single_mnrn_activity = exp( -(curr_time-prev_spike_time-20)*(curr_time-prev_spike_time-20)/(50+4.f*(curr_time-curr_spike_time)) );///70/100/					
					rest_activity = exp( -(curr_spike_time-prev_spike_time-0)*(curr_spike_time-prev_spike_time-0)/300 );
					single_mnrn_activity += exp( -(curr_time-curr_spike_time-20)*(curr_time-curr_spike_time-20)/(50+4.f*(curr_time-curr_spike_time)))*(1.f+rest_activity);///70/100/
					//single_mnrn_activity += 0;
				}
				else
				{
					single_mnrn_activity = exp( -(curr_time-curr_spike_time-20)*(curr_time-curr_spike_time-20)/(50+4.f*(curr_time-curr_spike_time)));///70/100/
					//single_mnrn_activity += 0;
				}

			}

			if(single_mnrn_activity>0)
			{
				/*
				if(sp<248)
				{
					//m_i= -1;
					m_i = (int) (0+ ((float)spike_pos[sp]-500.f)*23.f/2500.f );
					//m_i = (int) (50+ ((float)spike_pos[sp]-500.f)*23.f/2500.f );//mirror signals
				}
				else
				{
					//m_i= -1;
					m_i = (int) (50+ ((float)spike_pos[sp]-500.f)*23.f/2500.f );
					//m_i = (int) (0+ ((float)spike_pos[sp]-500.f)*23.f/2500.f );//mirror signals
					//m_i+=0;
				}*/

				if(spike_pos[sp]>= 500.00) m_i =  1;
				if(spike_pos[sp]>= 734.29) m_i =  2;
				if(spike_pos[sp]>= 905.09) m_i =  3;
				if(spike_pos[sp]>=1035.30) m_i =  4;
				if(spike_pos[sp]>=1160.44) m_i =  5;
				if(spike_pos[sp]>=1294.08) m_i =  6;
				if(spike_pos[sp]>=1441.16) m_i =  7;
				if(spike_pos[sp]>=1588.28) m_i =  8;
				if(spike_pos[sp]>=1732.02) m_i =  9;
				if(spike_pos[sp]>=1857.16) m_i = 10;
				if(spike_pos[sp]>=1982.30) m_i = 11;
				if(spike_pos[sp]>=2102.37) m_i = 12;
				if(spike_pos[sp]>=2222.43) m_i = 13;
				if(spike_pos[sp]>=2342.50) m_i = 14;
				if(spike_pos[sp]>=2462.57) m_i = 15;
				if(spike_pos[sp]>=2582.63) m_i = 16;
				if(spike_pos[sp]>=2685.79) m_i = 17;
				if(spike_pos[sp]>=2770.34) m_i = 18;
				if(spike_pos[sp]>=2854.89) m_i = 19;
				if(spike_pos[sp]>=2939.45) m_i = 20;

				//if(sp>=248) m_i += 50;
				if(sp<248) m_i += 50; // flip the tadpole muscles

					muscle_activation_signal_cpp[m_i+0] += (1.f/1.4f)*(single_mnrn_activity/10.f)*0.8f;//*0.5f;//1.f 2.f//12.f; //change max muscle force

					//muscle_activation_signal_cpp[m_i*2+1] += (single_mnrn_activity/10.f)*0.7f * 0;//2.f//12.f; //change max muscle force
					
			}
		}

		/**/
		for(m_i=1;m_i<45;m_i++)
		{
			muscle_activation_signal_cpp[	m_i] = min(muscle_activation_signal_cpp[   m_i]*0.930f/*355f*/, 1.f); // R
			muscle_activation_signal_cpp[50+m_i] = min(muscle_activation_signal_cpp[50+m_i]*1.0f, 1.f); // L
		}


		//}

		ocl_solver->updateMuscleActivityData(muscle_activation_signal_cpp);

		return helper->get_elapsedTime();
	}

	catch(std::exception &e)
	{
		std::cout << "ERROR_09: " << e.what() << std::endl;
		exit( -9 );
	}
}
/** Prepare data and log it to special configuration
 *  file you can run your simulation from place you snapshoted it
 *
 *  @param fileName - name of file where saved configuration will be stored
 */
void owPhysicsFluidSimulator::makeSnapshot(const std::string & fileName){
	getVelocity_cpp();
	owHelper::loadConfigurationToFile(position_cpp, velocity_cpp, elasticConnectionsData_cpp, membraneData_cpp, particleMembranesList_cpp, fileName.c_str(), config);
}

//Destructor
owPhysicsFluidSimulator::~owPhysicsFluidSimulator(void)
{
	delete [] position_cpp;
	delete [] velocity_cpp;
	delete [] density_cpp;
	delete [] particleIndex_cpp;
	if(numOfElasticP != 0)
		delete [] elasticConnectionsData_cpp;
	delete [] muscle_activation_signal_cpp;
	if(membraneData_cpp != NULL) {
		delete [] membraneData_cpp;
		delete [] particleMembranesList_cpp;
	}
	delete config;
	delete ocl_solver;
}
