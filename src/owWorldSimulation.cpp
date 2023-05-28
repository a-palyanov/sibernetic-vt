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

#include <stdio.h>
#include <sstream>
#include <csignal>

#include "owWorldSimulation.h"

#include "owWorldSimulation.h"

extern int numOfLiquidP;
extern int numOfElasticP;
extern int numOfBoundaryP;
extern int numOfMembranes;
extern bool load_from_file;
extern bool load_to;

int old_x=0, old_y=0;	// Used for mouse event
float camera_trans[] = {0, -0.3f, -8.f};
float camera_rot[]   = {90, 90, 0};
//float camera_rot[]   = {0, -90*1, 0};
float camera_trans_lag[] = {0, 0, -8.f};
float camera_rot_lag[] = {0, 0, 0};
const float inertia = 1.0f;
float modelView[16];
int buttonState = 0;

float sc = 0.035f;		//0.0145;//0.045;//0.07

float sc_scale = 1.0f;

Vector3D ort1(1,0,0),ort2(0,1,0),ort3(0,0,1);
GLsizei viewHeight, viewWidth;
int winIdMain;
int MUSCLE_COUNT = 100;//increase this value and modify corresponding code if you plan to add more than 10 muscles
double totalTime = 0;
int frames_counter = 0;
double calculationTime;
double renderTime;
double fps;
char device_full_name [1000];
double prevTime;
unsigned int * p_indexb;
float * d_cpp;
float * p_cpp;
float * v_cpp;
float * ec_cpp;
float * muscle_activation_signal_cpp;
float spike_pos[496];
float spike_time[496][50];
int   * md_cpp;// pointer to membraneData_cpp
owPhysicsFluidSimulator * fluid_simulation;
owHelper * helper;
owConfigProperty * localConfig;
bool flag = false;
bool sPause = false;
void * m_font = (void *) GLUT_BITMAP_9_BY_15;//GLUT_BITMAP_8_BY_13;
void * m_font2 = (void*) GLUT_BITMAP_TIMES_ROMAN_24;
int iteration = 0;
unsigned char* img_data = NULL;
int img_w = 0, img_h = 0;


void calculateFPS();
void drawScene(int view_type);
void renderInfo(int,int);
void glPrint(float,float,const char *, void*);
void glPrint3D(float,float,float,const char *, void*);
void Cleanup(int);
//float muscle_activation_signal [10] = {0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f};
void beginWinCoords(void)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(0.0f, (GLfloat)glutGet(GLUT_WINDOW_HEIGHT) - 10, 0.0f);
    glScalef(8.0f, -1.0f, 1.0f);
	
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, glutGet(GLUT_WINDOW_WIDTH), 0, glutGet(GLUT_WINDOW_HEIGHT), -1, 1);
	
    glMatrixMode(GL_MODELVIEW);
}

void endWinCoords(void)
{
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}
void glPrint(float x, float y, const char *s, void *font)
{
	glRasterPos2f((GLfloat)x, (GLfloat)y);
    int len = (int) strlen(s);
    for (int i = 0; i < len; i++) {
        glutBitmapCharacter(font, s[i]);
    }
}
void glPrint3D(float x, float y, float z, const char *s, void *font)
{
	glRasterPos3f((GLfloat)x, (GLfloat)y, (GLfloat)z);
    int len = (int) strlen(s);
    for (int i = 0; i < len; i++) {
        glutBitmapCharacter(font, s[i]);
    }
}
/** Main displaying function
 */

/*
float camera_trans[] = {0, -0.3, -8.0};
float camera_rot[]   = {90, 90, 0};
//float camera_rot[]   = {0, -90*1, 0};
float camera_trans_lag[] = {0, 0, -8.0};
float camera_rot_lag[] = {0, 0, 0};
const float inertia = 1.0f;
float modelView[16];
int buttonState = 0;

float sc = 0.035f;		//0.0145;//0.045;//0.07 
*/

int myCompare2( const void * v1, const void * v2 ){
	int * f1 = (int *)v1;
	int * f2 = (int *)v2;
	if( f1[ 0 ] < f2[ 0 ] ) return -1;
	if( f1[ 0 ] > f2[ 0 ] ) return +1;
	return 0;
}

	float   vel_matrix[(125+35+10)*5][(35+10)*5];
	int vel_matrix_cnt[(125+35+10)*5][(35+10)*5];
	int elast_p_index[15000*2];

void display(void)
{
	int n_elast_p;

	//Update Scene if not paused
	int i,j,k;
	int err_coord_cnt = 0;
	if(!sPause){
		if(load_from_file){
			owHelper::loadConfigurationFromFile(p_cpp,ec_cpp,md_cpp, localConfig,iteration);
			iteration++;
		}else{
			calculationTime = fluid_simulation->simulationStep(load_to); // Run one simulation step
			int pib;
			p_indexb = fluid_simulation->getParticleIndex_cpp();
			for(i=0;i<localConfig->getParticleCount();i++)
			{
				pib = p_indexb[2*i + 1];
				p_indexb[2*pib + 0] = i;
			}
			p_cpp = fluid_simulation->getPosition_cpp();
			d_cpp = fluid_simulation->getDensity_cpp();
			ec_cpp = fluid_simulation->getElasticConnectionsData_cpp();
			v_cpp = fluid_simulation->getVelocity_cpp();
			/*
			if(fluid_simulation->getIteration() == localConfig->getNumberOfIteration()){
				std::cout << "Simulation is reached time limit" << std::endl;
				Cleanup(EXIT_SUCCESS);
			}*/

		}
		helper->refreshTime();

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	}

	//int view_type = -1;
	//if(fluid_simulation->getIteration() % 10 == 0)

	// Display user interface if enabled
	bool displayInfos = true;
	
    if (displayInfos)
    {
        //glDisable(GL_DEPTH_TEST);
        //glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ZERO); // invert color
		//glDisable(GL_BLEND);
        renderInfo(0, 0);
        //glEnable(GL_BLEND);
		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        //glEnable(GL_DEPTH_TEST);
    }




	/*if(fluid_simulation->getIteration() == 1 )
	{
		
	}*/
	 
	//int view_type = 1;
	//if(0==1)
	for(int view_type = 0; view_type <=2; view_type++)
	{
	/**/
		sc = 0.0063f;//0.008
		if(view_type == 0) { glTranslatef( ( 3.3f+0.6f-0.6f)/6.f, 0.f, -0.19f ); }
		if(view_type == 1) { glTranslatef( (-5.0f-0.27f+0.25f-0.17f)/6.f, 0.f, 0.0f ); glRotatef(-90.f, 0.f, 0.f, 1.0f ); }
		if(view_type == 2) { glRotatef( 90.f, 0.f, 0.f, 1.0f ); glTranslatef( (1.7f-0.33f-0.25f+0.77f)/6.f, 0.f, 0.19f ); break; }
		//if(view_type == 1) { glTranslatef( -(3.4f-1.f)/6.f, 0.f, 0.30f ); break; }
	/**/

	//sPause = true;//start paused

	drawScene(view_type);
	
	//glBegin(GL_POINTS);
	float dc, rho;
	//Display all particles
	FILE* f_trajectory_log;
	float xc=0,yc=0,zc=0,nc=0;
	int p_type = 0;
	int nn = 0;// number of neighbours
	int ok_to_display;
	float abs_v;
	float max_abs_v = 0;
//	double v_mean = 0;
	float m_clr;
	int muscle_index;
	int muscle_index2;
	GLUquadricObj *quadObj = gluNewQuadric();
	
	for(i=0;i<(125+35+10)*5;i++)
	{
		for(j=0;j<(35+10)*5;j++)
		{
			vel_matrix[i][j] = 0;
			vel_matrix_cnt[i][j] = 0;
		}
	}
	

	if(view_type==0)
	{
		if(fluid_simulation->getIteration() == 1 ) 
		{
			f_trajectory_log = fopen("tadpole_trajectory.txt","wt");
			fclose(f_trajectory_log);
		}
	}

	// insert elastic particles sorting here!!! 22.03.2021
	/*
	for(i = 0; i < localConfig->get; i++)
	{

	}*/
	n_elast_p = 0;

	for(i=0;i<15000;i++)
	{
		elast_p_index[i*2+0] = -1000;
		elast_p_index[i*2+1] = -1;

		if( ((int)p_cpp[i*4+3])==ELASTIC_PARTICLE )	
		{
			elast_p_index[i*2+0] = -(int)(p_cpp[i*4+0]*1000.f);
			if(view_type==0) elast_p_index[i*2+0] = (int)(p_cpp[i*4+1]*1000.f);
			elast_p_index[i*2+1] = i;
			n_elast_p++;
		}
	}

	printf("n_elast_p = %d \n", n_elast_p);

	qsort( elast_p_index, n_elast_p, 2 * sizeof( int ), myCompare2 );

	// in case of too much transparency
	//for(int ii = localConfig->getParticleCount() - 1; ii >= 0 ; ii--)

	/*
	for(int ii = 0; ii < localConfig->getParticleCount(); ii++)
	{
					if(   (p_cpp[ii*4+0]>0)&&(p_cpp[ii*4+0]<localConfig->xmax-0*r0/2)
						&&(p_cpp[ii*4+1]>0)&&(p_cpp[ii*4+1]<localConfig->ymax-0*r0/2)
						&&(p_cpp[ii*4+2]>0)&&(p_cpp[ii*4+2]<localConfig->zmax-0*r0/2))
					{

					}
					else
					{
						printf("[%d]out_of_boundaries,",ii);
					}

	}*/


	for(int ii = 0; ii < localConfig->getParticleCount(); ii++)
	{
		/*
		if(view_type==0)
		{
			i = localConfig->getParticleCount() - 1 - ii;
		}
		else*/
		i = ii;
		
		if( ((int)p_cpp[ii*4+3])==ELASTIC_PARTICLE )	
		{
			i = elast_p_index[ii*2+1];
		}

		glPointSize(0.1f);
		if(!load_from_file)
		{
			rho = d_cpp[ p_indexb[ i * 2 + 0 ] ];
			if( rho < 0 ) rho = 0;
			if( rho > 2 * rho0) rho = 2 * rho0;
			dc = 20.f * ( rho - rho0 ) / rho0 ;
			if(dc>1.f) dc = 1.f;
			//  R   G   B
			if( ((int)p_cpp[i * 4 + 3]) == LIQUID_PARTICLE )
			{
			abs_v = 500.f/*500.f*/*sqrt(v_cpp[i*4+0]*v_cpp[i*4+0] + v_cpp[i*4+1]*v_cpp[i*4+1] + v_cpp[i*4+2]*v_cpp[i*4+2]);
			max_abs_v = std::max(max_abs_v,abs_v);
			}
			// 0, 0.4, 1.0, 0.5
			/*
			if( abs_v <  1.0f )	glColor4f(   0,  0,						1, 0.2f);//blue
			if( abs_v >= 1.0f )	glColor4f(   0,  std::maxabs_v-1,0.f),		1, 0.2f);//cyan
			if( abs_v >= 2.0f )	glColor4f(   0,   1, 1-std::maxabs_v-2.f,0.f), 0.2f);//green
			if( abs_v >= 3.0f )	glColor4f(  std::maxabs_v-3.f,0.f),    1,   0, 0.2f);//yellow
			if( abs_v >= 4.0f )	glColor4f(   1, 1-std::maxabs_v-4.f,0.f),   0, 0.2f);//red
			if( abs_v >= 5.0f )	glColor4f(   1-std::maxabs_v-5.f,0.f), 0,   0, 0.2f);//red
			if( abs_v >= 5.7f )	glColor4f(   0.3,				   0,   0, 0.2f);//red
			*/

			ok_to_display = 1;
			if(view_type==0) { /*if((p_cpp[i*4+1]-localConfig->ymax/2)<10.f) ok_to_display = 0;*/ } // for lower level of water, 0.65 instead of 0.8
			if(view_type==1) { /*if(	!(((p_cpp[i*4+0]-localConfig->ymax/2)>-35.f)&& //liquid's side walls
									((p_cpp[i*4+0]-localConfig->ymax/2)< 67.f))) ok_to_display = 0;*/ }
			if(ok_to_display)
			if( ((int)p_cpp[i * 4 + 3]) == LIQUID_PARTICLE )
			{
				if(view_type==0) 
				{
					/**/
					if(   (p_cpp[i*4+0]>r0*2.f)&&(p_cpp[i*4+0]<localConfig->xmax-r0*2.f)
						&&(p_cpp[i*4+1]>r0*2.f)&&(p_cpp[i*4+1]<localConfig->ymax-r0*2.f)
						&&(p_cpp[i*4+2]>r0*2.f)&&(p_cpp[i*4+2]<localConfig->zmax-r0*2.f))/**/
					{
							vel_matrix[std::min((160+15)*5-1,(int)(p_cpp[i*4+2]*5.f/h))][std::min((35+10)*5-1,(int)(p_cpp[i*4+0]*5.f/h))] += abs_v; 
						vel_matrix_cnt[std::min((160+15)*5-1,(int)(p_cpp[i*4+2]*5.f/h))][std::min((35+10)*5-1,(int)(p_cpp[i*4+0]*5.f/h))] ++;
					}
					else
					{
							vel_matrix[std::min((160+15)*5-1,(int)(p_cpp[i*4+2]*5.f/h))][std::min((35+10)*5-1,(int)(p_cpp[i*4+0]*5.f/h))] += 0; 
						vel_matrix_cnt[std::min((160+15)*5-1,(int)(p_cpp[i*4+2]*5.f/h))][std::min((35+10)*5-1,(int)(p_cpp[i*4+0]*5.f/h))] ++;
					}
				}

				if(view_type==1) 
				{
					if(   (p_cpp[i*4+0]>r0*2.f)&&(p_cpp[i*4+0]<localConfig->xmax-r0*2.f)
						&&(p_cpp[i*4+1]>r0*2.f)&&(p_cpp[i*4+1]<localConfig->ymax-r0*2.f)
						&&(p_cpp[i*4+2]>r0*2.f)&&(p_cpp[i*4+2]<localConfig->zmax-r0*2.f))
					{
							vel_matrix[std::min((160+15)*5-1,(int)(p_cpp[i*4+2]*5.f/h))][std::min(35*5-1,(int)(p_cpp[i*4+1]*5.f/h))] += abs_v;
						vel_matrix_cnt[std::min((160+15)*5-1,(int)(p_cpp[i*4+2]*5.f/h))][std::min(35*5-1,(int)(p_cpp[i*4+1]*5.f/h))] ++;
					}
					else
					{
							vel_matrix[std::min((160+15)*5-1,(int)(p_cpp[i*4+2]*5.f/h))][std::min(35*5-1,(int)(p_cpp[i*4+1]*5.f/h))] += 0;
						vel_matrix_cnt[std::min((160+15)*5-1,(int)(p_cpp[i*4+2]*5.f/h))][std::min(35*5-1,(int)(p_cpp[i*4+1]*5.f/h))] ++;
					}
				}
			}

			if(view_type==1) 
			{
				glColor4f(   0,  0,   1, 0.2f);//blue
			}

			//glColor4f( (abs_v>1)*std::min1.f,abs_v-1.f),  std::min1.f,abs_v),  1-(abs_v>1)*std::min1.f,abs_v-1.f), 0.5f/*0.5f/*0.3f /*0.1f*/);//blue
			//display liquid particles, blue

			if(!load_from_file){
				/*
				if( (dc=100*(rho-rho0*1.00f)/rho0) >0 )	glColor4f(   0,  dc,   1,0.2f);//cyan
				if( (dc=100*(rho-rho0*1.01f)/rho0) >0 )	glColor4f(   0,   1,1-dc,0.2f);//green
				if( (dc=100*(rho-rho0*1.02f)/rho0) >0 )	glColor4f(  dc,   1,   0,0.2f);//yellow
				if( (dc=100*(rho-rho0*1.03f)/rho0) >0 )	glColor4f(   1,1-dc,   0,0.2f);//red
				if( (dc=100*(rho-rho0*1.04f)/rho0) >0 )	glColor4f(   1,   0,   0,0.2f);

				//if((int)(p_cpp[i*4 + 3]*10)==12) glColor4f(   1,   0,   1,1.0f);
				//if (rho < 0.8f*rho0) glColor4f(0, 0, 0.5, 1.0f); // liquid_surface
				if( ((int)p_cpp[i * 4 + 3]) == LIQUID_PARTICLE )
				{
					glPointSize(1.f);

					// spheres instead of points 2020

					glBegin(GL_POINTS);
					glVertex3f( (p_cpp[i*4]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc );
					glEnd();
					//nn = ((int)(p_cpp[i * 4 + 3]*10000.f)) - ((int)(p_cpp[i * 4 + 3] * 100.f))*100;
					//if(nn<23) glColor4f(0, 0, 0.75, 0.4f); // liquid_surface
					//printf("\nnn = %d",nn);
					//nn += 0;
				}
				*/
			}
		}
		else
			glColor4f(  0,  0,  1, 1.0f);//blue
		
		if( !( ((int)p_cpp[i*4 + 3] == BOUNDARY_PARTICLE) /*&& (p_cpp[i*4 + 3] > 3.4f)*/  ) || ( (p_cpp[i*4 + 3] > 3.45) && (p_cpp[i*4 + 3] < 3.55) ) ) 
		/*&& (int)p_cpp[i*4 + 3] != ELASTIC_PARTICLE)*/
		//if( (p_cpp[i*4 + 3] > 2.45) && (p_cpp[i*4 + 3] < 2.55) )
		{
			
			

		//	if(p_cpp[i*4+3]<2.35)  glPointSize(3.f);
		//	if((int)p_cpp[i*4+3]==2)  glPointSize(3.f);
		//	if((int)p_cpp[i*4+3]==1)  glPointSize(0.5f);

			if((int)p_cpp[i*4+3]==2) // ELASTIC_PARTICLE
			{
				glPointSize(5.f);//3

				//glColor4f(   0,   1,   0,  1.0f );// color of elastic particles
				//glColor3ub(154,149,111); //tadpole body, elastic particles
				//glColor4b(92,85,39,255/2);
				glColor3ub(205+38*0,196+38*0,167-38*0); // tadpole body color 2020

				//if( (p_cpp[i*4  ]>=30.1f) ) /*&& (p_cpp[i*4  ]<=30.1f)*/ 
				//glColor4f(   0,     0, 100,  1.0f);// color of elastic particles

				if(p_cpp[i*4+3]<2.35)	
				{
					//glColor4b( 0, 0*155/2, 0, 255/2);/* tadpole belly*/
					//glColor4b( 92, 85, 39, 255/2);/* tadpole belly*/
					//glColor4b( 92/2, 85/2, 39/2, 255/2);/* tadpole belly*/
					//glColor3ub(154,149,111);
					glColor3ub( (GLubyte)(189*0.9+38*0),(GLubyte)(177*0.9+38*0),(GLubyte)(138*0.9-38*0) ); //tadpole belly color 2020
					
				}

				if(p_cpp[i*4+3]<2.31)	
				{
		//			glColor4b( 0, 0, 0, 255/2);/* tadpole eye, black*/
				}

				xc += p_cpp[i*4+0];
				yc += p_cpp[i*4+1];
				zc += p_cpp[i*4+2];
				nc+=1.f;
				
			}

			
			ok_to_display = 1;

			/*
			if((view_type == 0)&&( ((int)(p_cpp[i*4+3]*100))==239 )) 
			{
				ok_to_display = 2;
			}*/

			/*
			if( ((int)p_cpp[i*4+3])==LIQUID_PARTICLE )
			{
				if(view_type==0) { if((p_cpp[i*4+1]-localConfig->ymax/2)<10.f) ok_to_display = 0; } // for lower level of water, 0.65 instead of 0.8
				if(view_type==1) { if(	((p_cpp[i*4+0]-localConfig->ymax/2)>-35.f)&& 
										((p_cpp[i*4+0]-localConfig->ymax/2)< 67.f)) ok_to_display = 0; } //liquid's side walls
			}
			/**/

			/*
			�� ��������
			int i_ec = i*MAX_NEIGHBOR_COUNT;
			if( ec_cpp[ 4 * i_ec + 2 ] >= 1.f )//muscles
			{
				glColor4b( 255/2, 255/2, 0, 255/2);
			}*/
				

			if(ok_to_display)
			{
				
				if( ((int)p_cpp[i*4+3])==LIQUID_PARTICLE )	
				{
					//glPointSize(6.f);
					//if(view_type==1) 
						glPointSize(3.f);

				}
				//if(p_cpp[i*4+3]<2.35)
				
				/**/
				if( ((int)p_cpp[i*4+3])==ELASTIC_PARTICLE )	
				{
					//printf("%d, ", i);
					//example: positionVector[ 3 ] += 0.00042f;

					/*
					int j = 1473;
					int m_index_i = ((int)((p_cpp[ i*4 + 3 ]+0.0000001)*100000))%100;
					int m_index_j = ((int)((p_cpp[ j*4 + 3 ]+0.0000001)*100000))%100;

						if((i==1472))
						{
							printf("[%d][%d] %f\t%d,\t%f\t%d\n",i,j,p_cpp[ 4 * i + 3 ], m_index_i,p_cpp[ 4 * j + 3 ], m_index_j);
							printf("");
						}
					*/
					

					
					//glColor4ub(localConfig->tadpole_color_r[i], localConfig->tadpole_color_g[i], localConfig->tadpole_color_b[i],0.05f); 
					glColor4ub((GLubyte)localConfig->tadpole_color_r[i], (GLubyte)localConfig->tadpole_color_g[i], (GLubyte)localConfig->tadpole_color_b[i],(GLubyte)(25.f)); 
					//printf("%d\n",i);

					// make upper view tadpole semi-transparent too
					/*if(view_type == 0)
					{
						glColor3ub(localConfig->tadpole_color_r[i], localConfig->tadpole_color_g[i], localConfig->tadpole_color_b[i]); 
					}*/

					if((view_type == 0)&&( ((int)(p_cpp[i*4+3]*100))==239 )) 
					{
						//ok_to_display = 2;
						//glColor4ub(localConfig->tadpole_color_r[i], localConfig->tadpole_color_g[i], localConfig->tadpole_color_b[i],25.f); 
					}

					//if( ((int)(p_cpp[i*4+3]*100))==238 ) glColor3ub();

					/**/
					//if(0==1)
					if( ( muscle_index = ((int)((p_cpp[ i*4 + 3 ]+0.000001)*100000))%100 ) > 0 )
					{
						//printf("\n%f\t%d",p_cpp[i*4+3], ((int)((p_cpp[ i*4 + 3 ]+0.0000001)*100000))%100 );
						//printf(":");
						//glColor3ub( 255, 255, 0);

						//muscle_activation_signal_cpp[ muscle_index ] = (rand()%100)/100.f;

						m_clr = std::min(1.f,muscle_activation_signal_cpp[ muscle_index ]);
						//glColor3ub( 255, (int)(255.f*(1.f-m_clr)),   0);//yellow-red
						//glDisable(GL_BLEND);
						glColor4ub(	(GLubyte)(std::min(255.f, localConfig->tadpole_color_r[i]+50+m_clr*(255-localConfig->tadpole_color_r[i]))), 
									(GLubyte)(std::min(255.f, localConfig->tadpole_color_g[i]*(1-m_clr)+50+0.f*m_clr*(255-localConfig->tadpole_color_g[i]))),
									(GLubyte)(std::min(255.f, localConfig->tadpole_color_b[i]*(1-m_clr))), (GLubyte)((90+m_clr*160)) ); 
						if((muscle_index==1)||(muscle_index==51))
						glColor4ub(	(GLubyte)(std::min(255.0, 206*0.8+50+m_clr*(255-206*0.8))), 
									(GLubyte)(std::min(255.0, 200*0.8*(1-m_clr) + 50+0*(255-200*0.8))),
									(GLubyte)(std::min(255.0, 170*0.8*(1-m_clr))), (GLubyte)(90+m_clr*160)); 

						//glColor4ub( 255*m_clr, 150, 150, 55*m_clr);

						//if(0==1)
						/*
						if(view_type == 0) // for non-transpanent tadpole
						{
							glColor3ub(	std::min255, localConfig->tadpole_color_r[i]+50+m_clr*(255-localConfig->tadpole_color_r[i])), 
										std::min255, localConfig->tadpole_color_g[i]*(1-m_clr)+50+0.f*m_clr*(255-localConfig->tadpole_color_g[i])),
										std::min255, localConfig->tadpole_color_b[i]*(1-m_clr)) ); 
							if((muscle_index==1)||(muscle_index==51))
							glColor3ub(	std::min255, 206*0.8+50+m_clr*(255-206*0.8)), 
										std::min255, 200*0.8*(1-m_clr)+50+0*(255-200*0.8)),
										std::min255, 170*0.8*(1-m_clr)));
						}*/

						//glEnable(GL_BLEND);
						//if(muscle_index%2==1) glColor3ub( 180, (int)(255.f*(1.f-m_clr)),   0);//yellow-red

						if( i+1 <localConfig->getParticleCount() )
						{
							muscle_index2 = ((int)((p_cpp[ (i+1)*4 + 3 ]+0.0000001)*100000))%100;

							if( (muscle_index>=1) && (muscle_index2>=1) && ( muscle_index2 != muscle_index ) && 
								( ( muscle_index < 25 ) || (( muscle_index >= 51 )&&( muscle_index < 75 )) ) )
							{
								//glColor3ub( 155+38*0, 155+38*0, 55-38*0); // brown on yellow
								glColor4ub(	(GLubyte)(std::min(255.f, localConfig->tadpole_color_r[i]*0.7f)), 
											(GLubyte)(std::min(255.f, localConfig->tadpole_color_g[i]*0.7f)),
											(GLubyte)(std::min(255.f, localConfig->tadpole_color_b[i]*0.7f)), (GLubyte)(30+m_clr*220) ); 

								/*
								if(view_type == 0) // for non-transparency
								{
									glColor3ub(	std::min255, localConfig->tadpole_color_r[i]*0.7f), 
												std::min255, localConfig->tadpole_color_g[i]*0.7f),
												std::min255, localConfig->tadpole_color_b[i]*0.7f)); 
									//glColor4ub( 255*m_clr*0.7, 150*0.7, 150*0.7, 55*m_clr);
								}*/

								//glColor4b( 255/2, 255/2, 0, 255/2);
							}

							/*if( muscle_index==1 )
								glColor3ub( 255, 0, 150);*/
						}
					}
					/**/
						
						//glColor3ub( rand()%255, rand()%255, rand()%255);


					glPushMatrix();
					glTranslated( (p_cpp[i*4]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc );
					if(ok_to_display==1)
					{
						if( ((p_cpp[i*4+3]>2.329)&&(p_cpp[i*4+3]<2.331)) ) //notochord
						{

				/**/		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				/**/		glEnable(GL_BLEND);
							/*if(view_type==1)*/ glColor4ub( 0, 0, 0, /*100*/30);
							gluSphere(quadObj,1.3f*sc,6,6);
				/**/		glDisable(GL_BLEND);
						}
						else
						if((p_cpp[i*4+3]>/*2.295*/2.305)&&(p_cpp[i*4+3]<2.315)) // mouth
						{
				/**/		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				/**/		glEnable(GL_BLEND);
							glColor4ub( 0, 0, 0, 50);
							gluSphere(quadObj,1.3f*sc,6,6);
				/**/		glDisable(GL_BLEND);
						}
						else
						/*if((p_cpp[i*4+3]>2.339)&&(p_cpp[i*4+3]<2.341)) // fins
						{
							glColor3ub( 170, 0, 170);
							gluSphere(quadObj,1.3f*sc,6,6);
						}
						else*/
						if((p_cpp[i*4+3]>2.349)&&(p_cpp[i*4+3]<2.351)) // fins rigidity ribs
						{
						//	glColor3ub( 0, 0, 170);
						//	gluSphere(quadObj,1.3f*sc,6,6);
						}
						else
						{
							/*
							if( ( muscle_index = ((int)((p_cpp[ i*4 + 3 ]+0.0000001)*100000))%100 ) > 0 )
								gluSphere(quadObj,0.5f*sc,6,6);
							else
								gluSphere(quadObj,0.3f*sc,6,6);*/
								
				/**/		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				/**/		glEnable(GL_BLEND);
							//glColor4f(0, 0, 1, 0.05f);
							
							//glutWireSphere(1.0f*sc,6,6);
							gluSphere(quadObj,1.f*sc,6,6);
				/**/		glDisable(GL_BLEND);
						}
					}
					else 
					{	
						//gluSphere(quadObj,0.3f*sc,6,6);
						//glutWireSphere(0.2f*sc,6,6);
					}
					glPopMatrix();
				}/**/
			}

			
			glPointSize(3.f);

			if(!((p_cpp[i*4  ]>=0)&&(p_cpp[i*4  ]<=localConfig->xmax)&&
				 (p_cpp[i*4+1]>=0)&&(p_cpp[i*4+1]<=localConfig->ymax)&&
				 (p_cpp[i*4+2]>=0)&&(p_cpp[i*4+2]<=localConfig->zmax) ))
			{
				char label[250];
				beginWinCoords();
				glRasterPos2f (0.01F, 0.05F);
				if(err_coord_cnt<50){
				sprintf(label,"%d: %f , %f , %f",i,p_cpp[i*4  ],p_cpp[i*4+1],p_cpp[i*4+2]);
				glPrint( 0.f, (float)(50+err_coord_cnt*11), label, m_font);}
				if(err_coord_cnt==50) {
				glPrint( 0, (float)(50+err_coord_cnt*11), "............", m_font);}
				err_coord_cnt++;
				endWinCoords();
			}
		}
	} // end of cycle over all particles

	float abs_v1,abs_v2,abs_v3,abs_v4;
	int filled_cell1, filled_cell2, filled_cell3, filled_cell4;
	int n_filled_cells;
	/*	

	if(view_type==1) glColor4f(   1,  0, 0, 0.2f);
	if(view_type==0) glColor4f(   0,  0, 1, 0.2f);

	glBegin(GL_QUADS);
	glVertex3d(0,0,0 + 0.5*view_type);
	glVertex3d(0,1,0 + 0.5*view_type);
	glVertex3d(0,1,2 + 0.5*view_type);
	glVertex3d(0,0,2 + 0.5*view_type);
	glEnd();

	/**/
	//if(0==1)
	for(i=0;i<(125+35+10)*5-1;i++)
	{
		for(j=0;j<(35+10*(view_type==0))*5-1;j++)
		{
			filled_cell1 = filled_cell2 = filled_cell3 = filled_cell4 = 0;
			abs_v = abs_v1 = abs_v2 = abs_v3 = abs_v4 = 0;
			if(vel_matrix_cnt[i  ][j  ]>0) { abs_v1 = vel_matrix[i  ][j  ] / vel_matrix_cnt[i  ][j  ]; filled_cell1++; }
			if(vel_matrix_cnt[i+1][j  ]>0) { abs_v2 = vel_matrix[i+1][j  ] / vel_matrix_cnt[i+1][j  ]; filled_cell2++; }
			if(vel_matrix_cnt[i  ][j+1]>0) { abs_v3 = vel_matrix[i  ][j+1] / vel_matrix_cnt[i  ][j+1]; filled_cell3++; }
			if(vel_matrix_cnt[i+1][j+1]>0) { abs_v4 = vel_matrix[i+1][j+1] / vel_matrix_cnt[i+1][j+1]; filled_cell4++; }
			
			if(filled_cell1>0) abs_v += 0.75f*abs_v1*0.2f; 
			if(filled_cell2>0) abs_v += 0.75f*abs_v2*0.2f;
			if(filled_cell3>0) abs_v += 0.75f*abs_v3*0.2f; 
			if(filled_cell4>0) abs_v += 0.75f*abs_v4*0.2f;

			if( (n_filled_cells = filled_cell1 + filled_cell2 + filled_cell3 + filled_cell4) > 0)
			{
				abs_v /= (float)n_filled_cells;
			}

			glColor4f(   0,  0.8f,	1, 0.35f);//blue

			abs_v *= 1.20f;
			abs_v -= 1.25f;
			
			
			if( abs_v <  1.0f )	glColor4f(   0,  0.8f,					1, 0.35f);//blue
			if( abs_v >= 1.0f )	glColor4f(   0,  std::max(abs_v-1.f,0.8f),		1, 0.35f);//cyan
			if( abs_v >= 2.0f )	glColor4f(   0,   1, 1-std::max(abs_v-2.f,0.f), 0.35f);//green
			if( abs_v >= 3.0f )	glColor4f(  std::max(abs_v-3.f,0.f),    1,   0, 0.35f);//yellow
			if( abs_v >= 4.0f )	glColor4f(   1, 1-std::max(abs_v-4.f,0.f),   0, 0.35f);//red
			//if( abs_v >= 5.0f )	glColor4f(   1-std::maxabs_v-5.f,0.f), 0,   0, 0.15f);//red
			if( abs_v >= 5.0f )	glColor4f(   1.0,				   0,   0, 0.35f);//red*/

			abs_v += 1.25f;
			abs_v /= 1.20f;
			
		//	if( abs_v <  1.0f )	glColor4f(   0,  1.f,					1, 0.35f);//blue
		//	if( abs_v >= 1.0f )
		//	if( abs_v <  0.2f )	glColor4f(   0,  0.8f,					1, 0.35f);//blue
		//	else glColor4f( std::minabs_v-0.2f,1.f),  std::max0.8f - (abs_v-0.2f)/2.f,0),		1, 0.35f);

			//glBegin(GL_TRIANGLES);
			if((view_type==0)/*&&(filled_cell>0)*/)
			{
				//switched off 2021
				/**/
				glBegin(GL_QUADS);
				glVertex3d( (((float)(j  ))*h/5.f-localConfig->xmax/2)*sc, -0.50, (((float)(i  ))*h/5.f-localConfig->zmax/2)*sc );
				glVertex3d( (((float)(j+1))*h/5.f-localConfig->xmax/2)*sc, -0.50, (((float)(i  ))*h/5.f-localConfig->zmax/2)*sc );
				glVertex3d( (((float)(j+1))*h/5.f-localConfig->xmax/2)*sc, -0.50, (((float)(i+1))*h/5.f-localConfig->zmax/2)*sc );
				glVertex3d( (((float)(j  ))*h/5.f-localConfig->xmax/2)*sc, -0.50, (((float)(i+1))*h/5.f-localConfig->zmax/2)*sc ); 
				glEnd();
				/**/
			}
			

			//if(0==1)
			if((view_type==1)&&(n_filled_cells>0)) 
			{
				//printf("%d",n_filled_cells);
				glColor4f(   0,  0.8f,	1, 0.35f);//blue
			//	if( abs_v <  0.2f )	glColor4f(   0,  0.8f,					1, 0.35f);//blue
			//	else glColor4f( std::minabs_v-0.2f,1.f),  std::max0.8f - (abs_v-0.2f)/2.f,0),		1, 0.35f);
				/*if( abs_v <  1.0f )	glColor4f(   0,  1.f,					1, 0.35f);//blue
				if( abs_v >= 1.0f )	*/
			//	glColor4f(   0,  0.8,	1, 0.35f);//blue

				//switched off 2021
				/**/
				glBegin(GL_QUADS);
				glVertex3d(  0.4, (((float)(j  ))*h/5.f-localConfig->ymax/2)*sc, (((float)(i  ))*h/5.f-localConfig->zmax/2)*sc );
				glVertex3d(  0.4, (((float)(j  ))*h/5.f-localConfig->ymax/2)*sc, (((float)(i+1))*h/5.f-localConfig->zmax/2)*sc );
				glVertex3d(  0.4, (((float)(j+1))*h/5.f-localConfig->ymax/2)*sc, (((float)(i+1))*h/5.f-localConfig->zmax/2)*sc );
				glVertex3d(  0.4, (((float)(j+1))*h/5.f-localConfig->ymax/2)*sc, (((float)(i  ))*h/5.f-localConfig->zmax/2)*sc ); 
				glEnd();
				/**/
			}
		}
	}
	/**/		

	if(view_type==0)
	{
		float t_ms = ((float)fluid_simulation->getIteration())* localConfig->getTimeStep()*1000.f  + 1*25.f - 50.f;
		/**/
		//t_ms = 55;
		float el_length = 1.f; // electrode length
		float el_W = 2.5f;
		float el_w = 1.0f;
		float el_activity = 0;

		//glColor3f( 0.5f, 0.5f, 0.5f);

		if(t_ms<40) { el_length = t_ms / 40.f;			  el_W = 0.5f + 2.f*el_length; /*el_w = 1.0f*el_length;*/ }
		if(t_ms>65) { el_length = 1.f - ((t_ms-65)/25.f); el_W = 0.5f + 2.f*el_length; /*el_w = 1.0f*el_length;*/}

		if((t_ms>=30)&&(t_ms<50)) el_activity = 0 + (t_ms-30) / 20.f;
		if((t_ms>=50)&&(t_ms<55)) el_activity = 1.f;
		if((t_ms>=55)&&(t_ms<75)) el_activity = 1 - ((t_ms-55) / 20.f);

		if( (t_ms >= 0) && (t_ms < 90.f) )
		{
			glColor3ub((GLubyte)(170+20*el_activity),(GLubyte)(170-170*el_activity), (GLubyte)(170+85*el_activity));

			glPushMatrix();
			glTranslated(( localConfig->xmax/2.f)*sc*1.0f , (localConfig->ymax/4.f)*sc, -(localConfig->zmax*(0.33f-0*0.05f))*sc);
			glRotated(-90,0,1,0); 
			gluCylinder(quadObj, sc*el_W, sc*el_w, (localConfig->xmax/2)*sc*0.9*el_length, 16, 16);

			glPopMatrix();
		}
		/**/

		if(fluid_simulation->getIteration()%200==0)
		{
			if(nc>0) 
			{
				f_trajectory_log = fopen("tadpole_trajectory.txt","a+");
				fprintf(f_trajectory_log,"%f\t%f\t%f\t%f\n",localConfig->getTimeStep()*(float)fluid_simulation->getIteration(),
					simulationScale*xc/nc,simulationScale*yc/nc,simulationScale*zc/nc);
				
				fclose(f_trajectory_log);
			}
		}
	}
	

	glLineWidth((GLfloat)0.1);
	int ecc=0;//elastic connections counter;
	int is_muscle = 0;
	float pi = 3.14159f;
	//Display elastic connections
	//if(0==1)
	int num_of_elastic_neighbours = 0;
	Vector3D p;
	float p_length;
	int p_sp_cnt = 0;
	int nn_i, nn_j;

	/*
	for(int i_ec=0; i_ec < numOfElasticP * MAX_NEIGHBOR_COUNT; i_ec++)
	{
		if((int)ec_cpp[ 4 * i_ec + 0 ]>1) 
			num_of_elastic_neighbours++;
		
		if((int)ec_cpp[ 4 * i_ec + 0 ]<0) break;
	}*/

	int point_of_two_muscles_connection;
	int muscle_index1;

	for(int i_ec=0; i_ec < numOfElasticP * MAX_NEIGHBOR_COUNT; i_ec++)
	{
		i = (i_ec / MAX_NEIGHBOR_COUNT);

		if(i_ec%MAX_NEIGHBOR_COUNT==0)
		{
			num_of_elastic_neighbours = 0;
			for(int jj=0;jj<MAX_NEIGHBOR_COUNT;jj++)
			{
				if((int)ec_cpp[ 4 * (i_ec+jj) + 0 ]>=0) 
				{
					num_of_elastic_neighbours++;
				}
				else
					break;
			}

			num_of_elastic_neighbours += 0;
		}

		muscle_index1 = 0;
		point_of_two_muscles_connection = 0;

		//offset = 0
		if(0==1)
		if( (j=(int)ec_cpp[ 4 * i_ec + 0 ]) >=0 )
		{
			i = (i_ec / MAX_NEIGHBOR_COUNT);// + (generateInitialConfiguration!=1)*numOfBoundaryP;

			/*
			if( ec_cpp[ 4 * i_ec + 2 ] >= 1.f )//muscles
			{
				printf("%1");
				if(muscle_index1==0) 
				{
					printf("%2");
					muscle_index1 = ec_cpp[ 4 * i_ec + 2 ];
				}
				else
				if(muscle_index1 != ec_cpp[ 4 * i_ec + 2 ])
				{
					printf("%3");
					point_of_two_muscles_connection = 1;
				}
			}*/

			if(i<j)
			{
				glColor4b(150/2, 125/2, 0, 255/2/*alpha*/);
				if(ec_cpp[ 4 * i_ec + 2 ]>=1.f)//muscles
				{
					glLineWidth((GLfloat)3.0);
					//glLineWidth((GLfloat)6.0);
					//--------------------
					if(0==1) 
					{
						if(ec_cpp[4*i_ec+2]-floor(ec_cpp[4*i_ec+2])>0.55f)
						{
							//if(muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2])-1) ]>0.1)
							glLineWidth((GLfloat)6.0*muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2]))] ); //else glLineWidth((GLfloat)2.0);
							//glLineWidth((GLfloat)6.0);
							//glDisable(GL_BLEND);
							glColor4b(127/2, 127/2, 27/2, 255/2);/* muscle_number+0.5 <--> grey-yellow*/
							glBegin(GL_LINES);
							glVertex3f( (p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc );
							glColor4b(255/2, 255/2, 255/2, 255/2);
							glVertex3f( (p_cpp[j*4+0]-localConfig->xmax/2)*sc , (p_cpp[j*4+1]-localConfig->ymax/2)*sc, (p_cpp[j*4+2]-localConfig->zmax/2)*sc );
							glEnd();
							//glEnable(GL_BLEND);
						}
						else
						if(ec_cpp[4*i_ec+2]-floor(ec_cpp[4*i_ec+2])>0.45f)
						{
							//if(muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2])-1) ]>0.1)
							glLineWidth((GLfloat)6.0*muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2]))] ); //else glLineWidth((GLfloat)2.0);
							//glLineWidth((GLfloat)6.0);
							//glDisable(GL_BLEND);
							glColor4b(0, 127/2, 255/2, 255/2);/* muscle_number+0.5 <--> violet*/
							glBegin(GL_LINES);
							glVertex3f( (p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc );
							glColor4b(255/2, 255/2, 255/2, 255/2);
							glVertex3f( (p_cpp[j*4+0]-localConfig->xmax/2)*sc , (p_cpp[j*4+1]-localConfig->ymax/2)*sc, (p_cpp[j*4+2]-localConfig->zmax/2)*sc );
							glEnd();
							//glEnable(GL_BLEND);
						}
						else
						if(ec_cpp[4*i_ec+2]-floor(ec_cpp[4*i_ec+2])>0.35f)
						{
							//if(muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2])-1) ]>0.1)
							glLineWidth((GLfloat)6.0*muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2]))] ); //else glLineWidth((GLfloat)2.0);
							//glLineWidth((GLfloat)6.0);
							//glDisable(GL_BLEND);
							glColor4b(255/2, 0, 255/2, 255/2);/* muscle_number+0.4 <--> magenta*/
							glBegin(GL_LINES);
							glVertex3f( (p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc );
							glColor4b(255/2, 255/2, 255/2, 255/2);
							glVertex3f( (p_cpp[j*4+0]-localConfig->xmax/2)*sc , (p_cpp[j*4+1]-localConfig->ymax/2)*sc, (p_cpp[j*4+2]-localConfig->zmax/2)*sc );
							glEnd();
							//glEnable(GL_BLEND);
						}
						else
						if(ec_cpp[4*i_ec+2]-floor(ec_cpp[4*i_ec+2])>0.25f)
						{
							//if(muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2])-1) ]>0.1)
							glLineWidth((GLfloat)6.0*muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2]))] ); //else glLineWidth((GLfloat)2.0);
							//glLineWidth((GLfloat)6.0);
							//glDisable(GL_BLEND);
							glColor4b(255/2, 0/2, 0, 255/2);/* muscle_number+0.3 <--> red*/
							glBegin(GL_LINES);
							glVertex3f( (p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc );
							glColor4b(255/2, 255/2, 255/2, 255/2);
							glVertex3f( (p_cpp[j*4+0]-localConfig->xmax/2)*sc , (p_cpp[j*4+1]-localConfig->ymax/2)*sc, (p_cpp[j*4+2]-localConfig->zmax/2)*sc );
							glEnd();
							//glEnable(GL_BLEND);
					}
					else
					if(ec_cpp[4*i_ec+2]-floor(ec_cpp[4*i_ec+2])>0.15f)
					{
						//if(muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2])-1) ]>0.1)
						glLineWidth((GLfloat)6.0*muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2]))] ); //else glLineWidth((GLfloat)2.0);
						//glLineWidth((GLfloat)6.0);
						//glDisable(GL_BLEND);
						glColor4b(255/2, 127/2, 0, 255/2);/* muscle_number+0.2 <--> orange*/
						glBegin(GL_LINES);
						glVertex3f( (p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc );
						glColor4b(255/2, 255/2, 255/2, 255/2);
						glVertex3f( (p_cpp[j*4+0]-localConfig->xmax/2)*sc , (p_cpp[j*4+1]-localConfig->ymax/2)*sc, (p_cpp[j*4+2]-localConfig->zmax/2)*sc );
						glEnd();
						//glEnable(GL_BLEND);
					}
					else
					{
						glColor4b(255/2, 0,     0, 255/2);/* muscle_number+0.1 <--> red */
						
						//glDisable(GL_BLEND);
						glBegin(GL_LINES);
						glVertex3f( (p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc );
						glVertex3f( (p_cpp[j*4+0]-localConfig->xmax/2)*sc , (p_cpp[j*4+1]-localConfig->ymax/2)*sc, (p_cpp[j*4+2]-localConfig->zmax/2)*sc );
						glEnd();
						//glEnable(GL_BLEND);
					}

					}//--------------------

					/**/
					//drawing muscles 2020, transparency switched OFF
					//glDisable(GL_BLEND);//2020//
					p = Vector3D((p_cpp[j*4+0]-localConfig->xmax/2)*sc , (p_cpp[j*4+1]-localConfig->ymax/2)*sc, (p_cpp[j*4+2]-localConfig->zmax/2)*sc) - 
					    Vector3D((p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc);

					m_clr = std::min(1.f,muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2]))]);
					glColor4b( 255/2, (int)(255.f*(1.f-m_clr)/2.f),   0, 255/2 );//yellow-red
					//glColor4b( 255/2, 255/2, 0, 255/2);
					/*
					if(((int)ec_cpp[4 * i_ec + 2])%2==0) 
					{
						glColor3ub( 0, 255, (int)(255.f*(1.f-m_clr)) );//green-cyan
					}*/	
					/*
					//glBegin(GL_LINES);
					glPointSize(4);
					glBegin(GL_POINTS);
					glVertex3f( (p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc );
					glVertex3f( (p_cpp[j*4+0]-localConfig->xmax/2)*sc , (p_cpp[j*4+1]-localConfig->ymax/2)*sc, (p_cpp[j*4+2]-localConfig->zmax/2)*sc );
					glEnd();
					/**/

					/*
					glPushMatrix();
					glTranslated((p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc);
					glRotated(180.0f*(double)atan2(p.x,p.z)/pi,0,1,0); 
					glRotated(-180.0f*(double)atan2(p.y,sqrt(p.x*p.x+p.z*p.z))/pi,1,0,0); 
					gluCylinder(quadObj,0.45*sc,0.45*sc, p.length(), 6, 6);
					glPopMatrix();
					glEnable(GL_BLEND);//2020//
					/**/

					/* muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2]))], muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2]))]*/
				}
				else
				{//ordinary springs
					/**/
					p = Vector3D((p_cpp[j * 4 + 0] - localConfig->xmax / 2) * sc, (p_cpp[j * 4 + 1] - localConfig->ymax / 2) * sc, (p_cpp[j * 4 + 2] - localConfig->zmax / 2) * sc) -
						Vector3D((p_cpp[i * 4 + 0] - localConfig->xmax / 2) * sc, (p_cpp[i * 4 + 1] - localConfig->ymax / 2) * sc, (p_cpp[i * 4 + 2] - localConfig->zmax / 2) * sc);

					p_length = p.length();
					
					//int pc3 = ((int)(p_cpp[i * 4 + 3] * 1000));
					nn_i = ((int)(p_cpp[i * 4 + 3] * 10000.f)) - ((int)(p_cpp[i * 4 + 3] * 100.f)) * 100;
					nn_j = ((int)(p_cpp[i * 4 + 3] * 10000.f)) - ((int)(p_cpp[i * 4 + 3] * 100.f)) * 100;

					/*
					if(pc3%2>0)
					{
						p_sp_cnt++;
					}*/
					
					//if(num_of_elastic_neighbours<18)
					//if( ( (nn_i<24) && (nn_j<24) ) || ((p_cpp[i * 4 + 3] > 2.295) && (p_cpp[i * 4 + 3] < 2.305)) ) // draw outer shell of 3D objects
					//if( ((int)(p_cpp[i * 4 + 3]*1000))%2 )
					
					if(0==1) //ON/OFF ordinary springs 
					{
					glLineWidth((GLfloat)1.0);
					//glDisable(GL_BLEND);
					glBegin(GL_LINES);
											glColor4b(250/2, 250/2, 250/2, 255/2);
											
					if(p_cpp[i*4+3]>2.15)	
					{
						//glColor4b( 50/2, 125/2, 0, 255/2);
						glColor4b( 100/2, 250/2, 0/2, (GLubyte)(0.5*255/2));//light-green
						//if (num_of_elastic_neighbours > 18)
						//	glColor4b(250 / 2, 250 / 2, 0 / 2, 0.5 * 255 / 2);
					}
					
					if((p_cpp[i*4+3]>2.295)&&(p_cpp[i*4+3]<2.305))	glColor4b( 0/2, 0, 0/2, 255/2);/* tadpole eye, black*/ // 2.3
					if((p_cpp[i*4+3]>2.305)&&(p_cpp[i*4+3]<2.315))	
					{
							glColor4b( 100/2, 250/2, 0/2, 255/2);/* tadpole mouth, light-green*/ // 2.31
					}
					if((p_cpp[i*4+3]>2.315)&&(p_cpp[i*4+3]<2.325))	
					{
							//glColor4b( 70/2, 0/2, 150/2, 255/2);/* tadpole belly, dark blue*/ // 2.32
					}
					if((p_cpp[i*4+3]>2.325)&&(p_cpp[i*4+3]<2.335))	
					{
							glColor4b( (GLubyte)(100/2.8f), (GLubyte)(250/2.8f), (GLubyte)(75/2.8f), (GLubyte)(255/2));/* tadpole vertical midline, white*/ // 2.32
					}

					
					/**/glVertex3f( (p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc );

											glColor4b(250/2, 250/2, 250/2, 255/2);

					if(p_cpp[j*4+3]>2.15)	
					{
						//glColor4b( 50/2, 125/2, 0, 255/2);// 2020 ordinary springs
						glColor4b( 100/2, 250/2, 0/2, 255/4);//light-green
						//if (num_of_elastic_neighbours > 18)
						//	glColor4b(250 / 2, 250 / 2, 0 / 2, 0.5 * 255 / 2);
					}

					if((p_cpp[j*4+3]>2.295)&&(p_cpp[j*4+3]<2.305))	glColor4b( 0/2, 0, 0/2, 255/2);/* tadpole eye, black*/ // 2.3
					if((p_cpp[j*4+3]>2.305)&&(p_cpp[j*4+3]<2.315))	
					{
						glColor4b( 100/2, 250/2, 0/2, 255/2);/* tadpole mouth, light-green*/ // 2.31
					}
					if((p_cpp[j*4+3]>2.315)&&(p_cpp[j*4+3]<2.325))	
					{
							glColor4b( 70/2, 0/2, 150/2, 255/2);/* tadpole belly, dark blue*/ // 2.32
					}
					if((p_cpp[j*4+3]>2.325)&&(p_cpp[j*4+3]<2.335))	
					{
							glColor4b( (GLubyte)(100/2.8f), (GLubyte)(250/2.8f), (GLubyte)(75/2.8f), 255/2);/* tadpole vertical midline, white*/ // 2.32
					}
					/**/glVertex3f( (p_cpp[j*4+0]-localConfig->xmax/2)*sc , (p_cpp[j*4+1]-localConfig->ymax/2)*sc, (p_cpp[j*4+2]-localConfig->zmax/2)*sc );
					glEnd();
					/**/
					}

					if(0==1)
					{
						if(num_of_elastic_neighbours<18)
						{
							glDisable(GL_BLEND);
							Vector3D p = Vector3D((p_cpp[j*4+0]-localConfig->xmax/2)*sc , (p_cpp[j*4+1]-localConfig->ymax/2)*sc, (p_cpp[j*4+2]-localConfig->zmax/2)*sc) - 
										 Vector3D((p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc);
							glPushMatrix();
							glTranslated((p_cpp[i*4+0]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc);
							glRotated(180.0f*(double)atan2(p.x,p.z)/pi,0,1,0); 
							glRotated(-180.0f*(double)atan2(p.y,sqrt(p.x*p.x+p.z*p.z))/pi,1,0,0); 
							glColor4b( 100/2, 250/2, 0/2, 255/4);
							gluCylinder(quadObj, /*muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2]))], muscle_activation_signal_cpp[ (int)(floor( ec_cpp[4*i_ec+2]))]*/ 0.03*sc,0.03*sc, p.length(), 6, 6 );
							glPopMatrix();
							glEnable(GL_BLEND);
						}
					}
					/**/

					//glEnable(GL_BLEND);
					/**/
				}

					

				ecc++;
			}
		}

		/*
		if(point_of_two_muscles_connection>0)
		{
				glColor4b( 100, 70, 30, 100/2);
				glBegin(GL_POINTS);
						glVertex3f( (p_cpp[i*4]-localConfig->xmax/2)*sc , (p_cpp[i*4+1]-localConfig->ymax/2)*sc, (p_cpp[i*4+2]-localConfig->zmax/2)*sc );
				glEnd();
				printf("$");
		}*/

		p_sp_cnt += 0;
	}
	
	//GLUquadricObj *quadObj; 
	//quadObj = gluNewQuadric(); 
/*
	glLineWidth(1.f);
	glPushMatrix();
	glTranslated( (30-localConfig->xmax/2)*sc, (40-localConfig->ymax/2)*sc, (55-localConfig->zmax/2)*sc  );
	glutWireSphere(10.5f*sc,8,8);
	glPopMatrix();
*/
	// Draw membranes
	if(!load_from_file)
		md_cpp = fluid_simulation->getMembraneData_cpp();
	glColor4b(0, 200/2, 150/2, (GLbyte)(255)/*alpha*/);
	for(int i_m = 0; i_m < numOfMembranes; i_m++)
	{
		i = md_cpp [i_m*3+0];
		j = md_cpp [i_m*3+1];
		k = md_cpp [i_m*3+2];

		glBegin(GL_LINES);
		glVertex3f( ((p_cpp[i*4]+p_cpp[j*4]+4*p_cpp[k*4])/6-localConfig->xmax/2)*sc , ((p_cpp[i*4+1]+p_cpp[j*4+1]+4*p_cpp[k*4+1])/6-localConfig->ymax/2)*sc, ((p_cpp[i*4+2]+p_cpp[j*4+2]+4*p_cpp[k*4+2])/6-localConfig->zmax/2)*sc );
		glVertex3f( ((p_cpp[i*4]+p_cpp[k*4]+4*p_cpp[j*4])/6-localConfig->xmax/2)*sc , ((p_cpp[i*4+1]+p_cpp[k*4+1]+4*p_cpp[j*4+1])/6-localConfig->ymax/2)*sc, ((p_cpp[i*4+2]+p_cpp[k*4+2]+4*p_cpp[j*4+2])/6-localConfig->zmax/2)*sc );

		glVertex3f( ((p_cpp[i*4]+p_cpp[k*4]+4*p_cpp[j*4])/6-localConfig->xmax/2)*sc , ((p_cpp[i*4+1]+p_cpp[k*4+1]+4*p_cpp[j*4+1])/6-localConfig->ymax/2)*sc, ((p_cpp[i*4+2]+p_cpp[k*4+2]+4*p_cpp[j*4+2])/6-localConfig->zmax/2)*sc );
		glVertex3f( ((p_cpp[j*4]+p_cpp[k*4]+4*p_cpp[i*4])/6-localConfig->xmax/2)*sc , ((p_cpp[j*4+1]+p_cpp[k*4+1]+4*p_cpp[i*4+1])/6-localConfig->ymax/2)*sc, ((p_cpp[j*4+2]+p_cpp[k*4+2]+4*p_cpp[i*4+2])/6-localConfig->zmax/2)*sc );

		glVertex3f( ((p_cpp[j*4]+p_cpp[k*4]+4*p_cpp[i*4])/6-localConfig->xmax/2)*sc , ((p_cpp[j*4+1]+p_cpp[k*4+1]+4*p_cpp[i*4+1])/6-localConfig->ymax/2)*sc, ((p_cpp[j*4+2]+p_cpp[k*4+2]+4*p_cpp[i*4+2])/6-localConfig->zmax/2)*sc );
		glVertex3f( ((p_cpp[i*4]+p_cpp[j*4]+4*p_cpp[k*4])/6-localConfig->xmax/2)*sc , ((p_cpp[i*4+1]+p_cpp[j*4+1]+4*p_cpp[k*4+1])/6-localConfig->ymax/2)*sc, ((p_cpp[i*4+2]+p_cpp[j*4+2]+4*p_cpp[k*4+2])/6-localConfig->zmax/2)*sc );
		glEnd();
	}
	glLineWidth((GLfloat)1.0);

	} // cycle over view_type

			
	glutSwapBuffers();

	helper->watch_report("graphics: \t\t%9.3f ms\n====================================\n");
	renderTime = helper->get_elapsedTime();
	totalTime += calculationTime + renderTime;
	calculateFPS();
}
/** Drawing main scene and bounding box
 */
inline void drawScene(int view_type)
{
	//       [7]----[6]
	//      / |     /|
	//    [3]----[2] |
	//     | [4]--|-[5]
	//     | /    | /
	//    [0]----[1]
	Vector3D vcenter(0,0,0);
	Vector3D vbox[8];
	float s_v = 1 /(simulationScale);// = 1 m in simulation
	float order = 0;
	while(s_v >= 1){
		s_v /= 10;
		if(s_v < 1)
		{
			s_v *= 10;
			break;
		}
		++order;
	}
	vbox[0] = Vector3D(localConfig->xmin,localConfig->ymin,localConfig->zmin);
	vbox[1] = Vector3D(localConfig->xmax,localConfig->ymin,localConfig->zmin);
	vbox[2] = Vector3D(localConfig->xmax,localConfig->ymax,localConfig->zmin);
	vbox[3] = Vector3D(localConfig->xmin,localConfig->ymax,localConfig->zmin);
	vbox[4] = Vector3D(localConfig->xmin,localConfig->ymin,localConfig->zmax);
	vbox[5] = Vector3D(localConfig->xmax,localConfig->ymin,localConfig->zmax);
	vbox[6] = Vector3D(localConfig->xmax,localConfig->ymax,localConfig->zmax);
	vbox[7] = Vector3D(localConfig->xmin,localConfig->ymax,localConfig->zmax);

	
	glBegin(GL_LINES);
	/*sc *=10;
	glColor3ub(255, 0, 0);
	glVertex3d(vcenter.x,vcenter.y,vcenter.z);
	glVertex3d(vcenter.x+sc,vcenter.y,vcenter.z);
	glColor3ub(0,255, 0);
	glVertex3d(vcenter.x,vcenter.y,vcenter.z);
	glVertex3d(vcenter.x,vcenter.y+sc,vcenter.z);
	glColor3ub(0, 0, 255);
	glVertex3d(vcenter.x,vcenter.y,vcenter.z);
	glVertex3d(vcenter.x,vcenter.y,vcenter.z+sc);
	sc /=10;*/
	vcenter = Vector3D(-(localConfig->xmin+localConfig->xmax)/2,-(localConfig->ymin+localConfig->ymax)/2,-(localConfig->zmin+localConfig->zmax)/2);
	vcenter *= sc;
	Vector3D v1,v2,v3,v4,v5,v6,v7,v8;
	v1 = Vector3D( -localConfig->xmax/2, -localConfig->ymax/2, -localConfig->zmax/2)*sc;
	v2 = Vector3D(  localConfig->xmax/2, -localConfig->ymax/2, -localConfig->zmax/2)*sc;
	v3 = Vector3D(  localConfig->xmax/2,  localConfig->ymax/2, -localConfig->zmax/2)*sc;
	v4 = Vector3D( -localConfig->xmax/2,  localConfig->ymax/2, -localConfig->zmax/2)*sc;
	v5 = Vector3D( -localConfig->xmax/2, -localConfig->ymax/2,  localConfig->zmax/2)*sc;
	v6 = Vector3D(  localConfig->xmax/2, -localConfig->ymax/2,  localConfig->zmax/2)*sc;
	v7 = Vector3D(  localConfig->xmax/2,  localConfig->ymax/2,  localConfig->zmax/2)*sc;
	v8 = Vector3D( -localConfig->xmax/2,  localConfig->ymax/2,  localConfig->zmax/2)*sc;
	//glColor3ub(255,255,255);//white
	glColor3ub( 70, 70, 70);//black
	glVertex3d(v1.x,v1.y,v1.z);
	glVertex3d(v2.x,v2.y,v2.z);


	//glColor3ub(255,255,255);//yellow
	glColor3ub( 70, 70, 70);//black
	glVertex3d(v2.x,v2.y,v2.z);
	glVertex3d(v3.x,v3.y,v3.z);

	glVertex3d(v3.x,v3.y,v3.z);
	glVertex3d(v4.x,v4.y,v4.z);

	glVertex3d(v4.x,v4.y,v4.z); //glColor3ub(0,255,0);//green
	glVertex3d(v1.x,v1.y,v1.z);

	//glColor3ub(0,0,255);//blue
	glVertex3d(v1.x,v1.y,v1.z); //glColor3ub(255,255,0);//yellow
	glVertex3d(v5.x,v5.y,v5.z);

	glVertex3d(v2.x,v2.y,v2.z);
	glVertex3d(v6.x,v6.y,v6.z);

	glVertex3d(v3.x,v3.y,v3.z);
	glVertex3d(v7.x,v7.y,v7.z);

	glVertex3d(v4.x,v4.y,v4.z);
	glVertex3d(v8.x,v8.y,v8.z);

	glVertex3d(v5.x,v5.y,v5.z);
	glVertex3d(v6.x,v6.y,v6.z);

	glVertex3d(v6.x,v6.y,v6.z);
	glVertex3d(v7.x,v7.y,v7.z);

	glVertex3d(v7.x,v7.y,v7.z);
	glVertex3d(v8.x + s_v*sc,v8.y,v8.z);

	glVertex3d(v8.x,v8.y,v8.z);
	glVertex3d(v5.x,v5.y,v5.z);
	glEnd();
	//
	glBegin(GL_LINES);
	glColor3ub(0,0,0);//black


	Vector3D v_s = Vector3D(  -localConfig->xmax/2 + s_v,  localConfig->ymax/2,  localConfig->zmax/2)*sc;
	glVertex3d(v_s.x, v_s.y, v_s.z);
	glVertex3d(v_s.x, v_s.y - 0.5f * sc , v_s.z);
	glLineWidth((GLfloat)10.0);
	glVertex3d( v8.x,  v8.y,  v8.z);
	glVertex3d(v_s.x, v_s.y, v_s.z);

	glEnd();
	glLineWidth((GLfloat)1.0);
	std::stringstream ss;
	std::string s;
	ss << order;
	s = "1E-" + ss.str() + "m";
	//glPrint3D( (float)v8.x + 0.4f*sc , (float)v8.y - 2.f * sc, (float)v8.z, "0", m_font);
	//glPrint3D( (float)v_s.x , (float)v_s.y - 2.f * sc, (float)v_s.z, s.c_str(), m_font);
	ss.str("");
	while(v_s.x < localConfig->xmax/2*sc){
		v_s.x += s_v * sc;
		if(v_s.x < localConfig->xmax/2*sc){
			glBegin(GL_LINES);
				glVertex3d(v_s.x, v_s.y, v_s.z);
				glVertex3d(v_s.x, v_s.y - 0.5f * sc , v_s.z);
			glEnd();
		}
	}
}

int count_s = 0;
float current_sv ;
static char label[1000];                            /* Storage for current string   */
bool showInfo = true;
bool showRuler = false;
/** Render addition test information
 */

void renderInfo(int x, int y)
{
	beginWinCoords();
	int y_m = y;
	int i_shift = 0;
	float *ma = muscle_activation_signal_cpp;

	if(showInfo){
		
		sprintf(label,"Liquid particles: %d, elastic matter particles: %d, boundary particles: %d; total count: %d", numOfLiquidP,
																													 numOfElasticP,
																													 numOfBoundaryP,localConfig->getParticleCount());
		glColor3b (0, 0, 0);
		glPrint( 2 , 10 , label, m_font);

		
		if(load_from_file)
			sprintf(label,"Selected device: %s, FPS = %.2f, time step: %d (%f s)", device_full_name+7, fps, iteration, iteration * localConfig->getTimeStep());
		else
			sprintf(label,"Selected device: %s, FPS = %.2f, time step: %d", device_full_name+7, fps, fluid_simulation->getIteration());
		//glColor3f (1.f, 1.f, 1.f);
		glPrint( 2 , 25 , label, m_font);
		
		sprintf(label, "Time: %.3f ms,  motoneuron activity from CNS Model mns-data_49",
			 ((float)fluid_simulation->getIteration())* localConfig->getTimeStep()*1000.f  + 1*25.f - 50.f 
			/*(((float)fluid_simulation->getIteration())* localConfig->getTimeStep()*1000.f  + 1*25.f - 50.f)/1000.f*/ );//time shift off
		glPrint(28.53f, 76, label, m_font2);

		float t_ms = ((float)fluid_simulation->getIteration())* localConfig->getTimeStep()*1000.f  + 1*25.f - 50.f;
/*		
		if((t_ms>=0.f)&&(t_ms<=4.f))
		{
			sprintf(label, "[stimulation]");
			glPrint(28, 395, label, m_font2);
		}
*/
		// scale bar
		glLineWidth(5);
		glColor3b(0,0,0);
		glBegin(GL_LINES);
			glVertex2f(1.5f		+1.5f,360+80);
			glVertex2f(6+3.98f	+1.5f,360+80);
		glEnd();

		//gravity and arrow
		sprintf(label, "1 mm");
		glPrint(11	+1.5f, 367+80, label, m_font2);

		glLineWidth(2);
		glBegin(GL_LINES);
			glVertex2f(5.5f	+25.f,383+15+35);
			glVertex2f(5.5f	+25.f,407+15+35);
		glEnd();
		glLineWidth(1.5);
		glBegin(GL_LINES);
			glVertex2f(4.8f	+25.f,395+15+40);
			glVertex2f(5.5f	+25.f,405+15+40);
		glEnd();
		glBegin(GL_LINES);
			glVertex2f(6.2f	+25.f,395+15+40);
			glVertex2f(5.5f	+25.f,405+15+40);
		glEnd();

		sprintf(label, "g");
		glPrint(7			+25.f, 367+80, label, m_font2);

		glLineWidth(1);

		if(1){
			i_shift = 0;
			

			sprintf(label, "  Right muscles activity [1-20]: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
				ma[ 1],ma[ 2],ma[ 3],ma[ 4],ma[ 5],ma[ 6],ma[ 7],ma[ 8],ma[ 9],ma[10],ma[11],ma[12],ma[13],
				ma[14],ma[15],ma[16],ma[17],ma[18],ma[19],ma[20]);
			glPrint( 0 , 54 , label, m_font);
			sprintf(label, "  Left  muscles activity [1-20]: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
				ma[51],ma[52],ma[53],ma[54],ma[55],ma[56],ma[57],ma[58],ma[59],ma[60],ma[61],ma[62],ma[63],
				ma[64],ma[65],ma[66],ma[67],ma[68],ma[69],ma[70]);
			glPrint( 0 , 40 , label, m_font);

			y_m = 40;
		}
	}
	if(showRuler){
		glColor3ub(255, 0, 0);
		float accuracy = 100;//what is it?
		float s_v = 1 * sc_scale * (1 /( accuracy * simulationScale));
		float s_v_10 = s_v / 10;
		std::stringstream ss;
		std::string s;
		glBegin(GL_LINES);
			glColor3f(1.0f,0.0f,0.0f);
			glVertex2f((GLfloat) 0.f,(GLfloat)y_m );
			glVertex2f((GLfloat) s_v,(GLfloat)y_m );
			glVertex2f((GLfloat) s_v,(GLfloat)y_m );
			glVertex2f((GLfloat) s_v,(GLfloat)y_m + 5.f );
		glEnd();
			glPrint( s_v , y_m + 15.f , "1E-02 m", m_font);
		glBegin(GL_LINES);
			glVertex2f((GLfloat) s_v_10,(GLfloat)y_m + 0.f);
			glVertex2f((GLfloat) s_v_10,(GLfloat)y_m + 5.f);
		glEnd();
		if( 8 * s_v/pow(10.f,count_s) >= glutGet(GLUT_WINDOW_WIDTH)/2 ){
			count_s++;
			flag = true;
		}else{
			if(count_s != 0 && 8 * s_v/pow(10.f,count_s - 1) < glutGet(GLUT_WINDOW_WIDTH)/2){
				//flag = false;
				count_s--;
			}
		}
		if(flag){
			for(int i = 1;i <= count_s; i++){
				glBegin(GL_LINES);
					glVertex2f((GLfloat) s_v/pow(10.f,i + 1),(GLfloat)y_m + 0.f);
					glVertex2f((GLfloat) s_v/pow(10.f,i + 1),(GLfloat)y_m + 5.f);
				glEnd();
				ss << i + 1 + 2;
				s = "1E-" + ss.str() + "m";
				ss.str("");
				glPrint( s_v/pow(10.f,i + 1) , y_m + 15.f , s.c_str(), m_font);
			}
		}
	}
	endWinCoords();
}
/** Calculation of FPS
 */
void calculateFPS()
{
    //  Increase frame count
	frames_counter++;
    int timeInterval = (int)(totalTime - prevTime);
    if(timeInterval >= 1000)
    {
		fps = frames_counter / (timeInterval / 1000.0f);
        prevTime = totalTime;
        frames_counter = 0;
		//printf("FPS: \t\t%9.3f fps\n====================================\n",	fps );
    }
}
void respond_mouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON)
		buttonState = 1;
	if (button == GLUT_RIGHT_BUTTON)
		buttonState = 3;
	int mods;
	mods = glutGetModifiers();
    if (mods & GLUT_ACTIVE_CTRL)
    {
        buttonState = 2;
    }
	if(state == GLUT_UP)
		buttonState = 0;
	old_x=x;
	old_y=y;
	if (button == 3)     // mouse wheel up
    {
        sc *= 1.1f;		 // Zoom in
		sc_scale *= 1.1f;// Zoom in
    }
    else
	if (button == 4)	 // mouse wheel down
    {
        sc /= 1.1f;		 // Zoom out
		sc_scale /= 1.1f;// Zoom out
    }
}

// GLUT callback
// called on mouse movement

void mouse_motion (int x, int y)
{
	float dx,dy;
	dy = (float)(y - old_y);
	dx = (float)(x - old_x);

	if(buttonState == 1)
	{
		camera_rot[0] += dy / 5.0f;
		camera_rot[1] += dx / 5.0f;
	}
	if(buttonState == 3){
		// middle = translate
		camera_trans[0] += dx / 100.0f;
		camera_trans[1] -= dy / 100.0f;
		//camera_trans[2] += (dy / 100.0f) * 0.5f * fabs(camera_trans[2]);
	}
	if(buttonState == 2){
		// middle = translate
		//camera_trans[0] += dx / 100.0f;
		//camera_trans[1] -= dy / 100.0f;
		camera_trans[2] += (dy / 100.0f) * 0.5f * fabs(camera_trans[2]);
	}
	old_x=x;
	old_y=y;
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	for (int c = 0; c < 3; ++c)
	{
	    camera_trans_lag[c] += (camera_trans[c] - camera_trans_lag[c]) * inertia;
		camera_rot_lag[c] += (camera_rot[c] - camera_rot_lag[c]) * inertia;
	}
    glTranslatef(camera_trans_lag[0], camera_trans_lag[1], camera_trans_lag[2]);
	glRotatef(camera_rot_lag[0], 1.0, 0.0, 0.0);
	glRotatef(camera_rot_lag[1], 0.0, 1.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, modelView);
}

extern float *muscle_activation_signal_cpp;
void Cleanup(int exitCode){
	delete fluid_simulation;
	delete helper;
	exit (exitCode);
	if(img_data!=NULL) delete [] img_data;
}
void RespondKey(unsigned char key, int x, int y)
{
	switch(key)
	{
	case '1':
		localConfig->setCofigFileName("demo1");
		helper->refreshTime();
		fluid_simulation->reset();
		sPause = false;
		break;
	case '2':
		//owHelper::configFileName = "demo2";
		localConfig->setCofigFileName("demo2");
		helper->refreshTime();
		fluid_simulation->reset();
		sPause = false;
		break;
	case '\033':// Escape quits
	case 'Q':   // Q quits
	case 'q':   // q quits
		Cleanup(EXIT_SUCCESS);
		break;
	case ' ':
		sPause = !sPause;
		std::cout << "\nSimulation Is Paused" << std::endl;
		break;
	case 's':
		fluid_simulation->makeSnapshot();
		break;
	}

	if(key == 'i')
	{
		showInfo = !showInfo;
	}
	if(key == 'r')
	{
		showRuler = !showRuler;
	}
	glutPostRedisplay();
}

//Auxiliary function
/** There can be only one idle() callback function. In an
 *  animation, this idle() function must update not only the
 *  main window but also all derived subwindows
 */
void idle (void)
{
  glutSetWindow (winIdMain);
  glutPostRedisplay ();
}
//static char label[1000];                            /* Storage for current string   */

void Timer(int value)
{
	// Re-register for next callback
    glutPostRedisplay();
    glutTimerFunc(TIMER_INTERVAL*0, Timer, 0);
}

GLvoid resize(GLsizei width, GLsizei height){

	if(height == 0) { height = 1; }
	if(width == 0) { width = 1; }

	glViewport(0, 0, width, height);					// Set view area
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	float aspectRatio = (GLfloat)width / (GLfloat)height;
	if (aspectRatio>1.f)
	{
		//glFrustum(-1*aspectRatio, 1*aspectRatio, -1, 1, 3, 45);
		glOrtho(-1*aspectRatio, 1*aspectRatio, -1, 1, 3, 45);
		  ////glOrtho(0, glutGet(GLUT_WINDOW_WIDTH), 0, glutGet(GLUT_WINDOW_HEIGHT), -1, 1);
	}
	else
	{
		//glFrustum(-1, 1, -1/aspectRatio, 1/aspectRatio, 3, 45);
		glOrtho(-1, 1, -1/aspectRatio, 1/aspectRatio, 3, 45);
		  ////glOrtho(0, glutGet(GLUT_WINDOW_WIDTH), 0, glutGet(GLUT_WINDOW_HEIGHT), -1, 1);
	}
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();



	for (int c = 0; c < 3; ++c)
	{
	    camera_trans_lag[c] += (camera_trans[c] - camera_trans_lag[c]) * inertia;
		camera_rot_lag[c] += (camera_rot[c] - camera_rot_lag[c]) * inertia;
	}
    glTranslatef(camera_trans_lag[0], camera_trans_lag[1], camera_trans_lag[2]);
	glRotatef(camera_rot_lag[0], 1.0, 0.0, 0.0);
	glRotatef(camera_rot_lag[1], 0.0, 1.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, modelView);
}
inline void init(void){
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_NORMALIZE);
	glEnable(GL_AUTO_NORMAL);

	GLfloat light_position[] = { -50.0, 50.0, 50.0, 0.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	GLfloat light_ambient[] = { 0.6f, 0.6f, 0.6f, 1.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_ambient);

	float ambient[4] = {0.6f, 0.6f, 0.6f, 1};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
	//glClearColor(0.7f, 0.7f, 0.7f, 1.0f);//background color
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
	glClearDepth(1.0f);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

//	glEnable(GL_BLEND);
//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

}
void sighandler(int s){
	std::cerr << "\nCaught signal CTRL+C. Exit Simulation..." << "\n"; // this is undefined behaviour should check signal value
	Cleanup(EXIT_SUCCESS);
}
/** Init & start simulation and graphic component if with_graphics==true
 *
 * 	@param argc
 * 	command line arguments going throw the main function
 * 	@param with_graphics
 * 	Flag indicates that simulation will be run with graphic or not
 * 	@param load_to
 * 	Flag indicates that simulation will in "load configuration to file" mode
 */
void run(int argc, char** argv, const bool with_graphics)
{
	helper = new owHelper();
	if(!load_from_file){
		fluid_simulation = new owPhysicsFluidSimulator(helper, argc, argv);
		localConfig = fluid_simulation->getConfig();
	}
	else{
		localConfig = new owConfigProperty(argc, argv);
		muscle_activation_signal_cpp = new float [MUSCLE_COUNT];
		for(int i=0;i<MUSCLE_COUNT;i++)
		{
			muscle_activation_signal_cpp[i] = 0.f;
		}
	}
	std::signal(SIGINT,sighandler);
	if(with_graphics){
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
		glutInitWindowSize(1400, 700);
		glutInitWindowPosition(100, 100);
		winIdMain = glutCreateWindow("Sibernetic-VT, special edition of the Sibernetic (sibernetic.org) for 3D X. laevis tadpole swimming simulation (c) 2020-2021 Andrey Palyanov");
		glutIdleFunc (idle);
		//Init physic Simulation
		init();
		glutDisplayFunc(display);
		glutReshapeFunc(resize);
		glutMouseFunc(respond_mouse);
		glutMotionFunc(mouse_motion);	//process movement in case if the mouse is clicked,
		glutKeyboardFunc(RespondKey);
		glutTimerFunc(TIMER_INTERVAL * 0, Timer, 0);
		glutMainLoop();
		if(!load_from_file){
			Cleanup(EXIT_SUCCESS);
		}
	}else{
		while(1){
			fluid_simulation->simulationStep(load_to);
			helper->refreshTime();
			if(load_to && fluid_simulation->getIteration() == 40000){
				delete fluid_simulation;
				delete helper;
				exit(EXIT_SUCCESS);
			}
		}
	}
    exit(EXIT_SUCCESS);
}
