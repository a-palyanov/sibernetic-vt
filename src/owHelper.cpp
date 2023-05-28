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

#include <string>
#include <stdio.h>
#include <sstream>
#include <string>
#include <vector>


#if defined(__APPLE__) || defined (__MACOSX)
#include <mach/mach.h>
#include <mach/mach_time.h>
#endif

#define EXPEREMENTAL_WRITE 0
#if EXPEREMENTAL_WRITE
#define EXPEREMENTAL_READ
#endif
#include "owHelper.h"
#include "owPhysicsConstant.h"

using namespace std;

extern int numOfMembranes;
extern int numOfElasticP;
extern int numOfLiquidP;


/** owHelpre class constructor
 */
owHelper::owHelper(void)
{
	refreshTime();
}
/** owHelpre class destructor
 */
owHelper::~owHelper(void)
{
}
/** Update time variable
 *
 *  Using precision system functions for getting exact system time.
 *  Initialization of start time value.
 *  For more info:
 *  Windows QueryPerformanceFrequency - http://msdn.microsoft.com/ru-ru/library/windows/desktop/ms644905(v=vs.85).aspx
 *  Linux clock_gettime - http://man7.org/linux/man-pages/man2/clock_gettime.2.html
 *  MacOS - https://developer.apple.com/library/mac/documentation/Darwin/Conceptual/KernelProgramming/services/services.html
 */
void owHelper::refreshTime()
{
#if defined(_WIN32) || defined (_WIN64)
    QueryPerformanceFrequency(&frequency);	// get ticks per second
    QueryPerformanceCounter(&t1);			// start timer
	t0 = t1;
#elif defined(__linux__)
	clock_gettime(CLOCK_MONOTONIC_RAW, &t1);
	t0 = t1;
#elif defined(__APPLE__)
    t1 = mach_absolute_time();
    t0 = t1;
#endif
}

extern unsigned char* img_data;
extern int img_w, img_h;

int generateTadpoleBody(int stage, int i_start,float *position_cpp, float *velocity_cpp, int &numOfMembranes, int *membraneData_cpp, owConfigProperty * config)
{
//	int i;
	int pCount = 0;
	int ycnt = 0;
	int xcnt = 0;
	int zcnt = 0;
	float dr2,dr02=r0*r0,min_dr2;
	min_dr2 = 1000000;
	float sqrt3 = sqrt(3.f);
	float sqrt2d3 = sqrt(2.f/3.f);
	float r0s = r0*0.48f*1.f;
	float x,y,z;
	float *positionVector;
	float *velocityVector;
	//float xc = 30, yc = 25, zc = 17;
	float xx,yy,zz;
	float particle_type;
	int within_body_shell = 0;
	int z_shift;
	float c_bmp, z_bmp, y_bmp;
	float tmp_x;
	int is_muscle;

	if(stage==0)
	{
		config->tadpole_y_min = 1000;
		config->tadpole_y_max =    0;
		config->tadpole_z_min = 1000;
		config->tadpole_z_max =    0;
	}

	y = config->ymax/2.f;
	x = config->xmax/2.f;
	z = config->zmax/2.f;
/*
					if(stage==1)
					{	
						positionVector = position_cpp + 4 * (pCount+i_start);
						positionVector[ 0 ] = x;
						positionVector[ 1 ] = y;
						positionVector[ 2 ] = z;
						positionVector[ 3 ] = 2.4f;
					}

					pCount++;

	return pCount;
/**/
/**
	//return 0;

	for(x= -r0s*2.f*sqrt2d3*11  + config->xmax/2.f,xcnt=0; x<config->xmax/2.f + r0s*2.f*sqrt2d3*11 + 0.001f; x+=r0s*2.f*sqrt2d3, xcnt++)	
	{
		for(y=config->ymax*0.475f/4,ycnt=0;y<config->ymax*0.98f-1.4f*r0;y+=r0s*sqrt3,ycnt++)//ok
		{
			for(z= config->zmax*0.6f+(r0s)*(ycnt%2==1)+(r0s*2.f/sqrt3)*(xcnt%2==1),zcnt=0;z<config->zmax*1.0f - 1.0f*r0;z+=2*r0s,zcnt++) //ok				
			{
				within_body_shell = -1;


				//if( (xx-11)*(xx-11) + (yy-16)*(yy-16) + (zz-265)*(zz-265) < 52 ) within_body_shell=1;//sphere								//190/4
				if( (x-config->xmax/2)*(x-config->xmax/2) + (y-r0s-config->ymax/2)*(y-r0s-config->ymax/2) + (z-300-100)*(z-300-100) < 30.f ) within_body_shell=1;//sphere, marble

				if(within_body_shell>0)
				{
					if(stage==1)
					{	
						positionVector = position_cpp + 4 * (pCount+i_start);
						positionVector[ 0 ] = x;
						positionVector[ 1 ] = y-10;
						positionVector[ 2 ] = z-300;//+80+1*280;// + (r0s*2.f/sqrt3)*(xcnt%2==1);
						positionVector[ 3 ] = 2.4f;
					}

					pCount++;
				}
			}
		}
	}

	return pCount;
/**/
	//printf();
		
		for(x= -r0s*2.f*sqrt2d3*11  + config->xmax/2.f,xcnt=0; x<config->xmax/2.f + r0s*2.f*sqrt2d3*11 + 0.001f; x+=r0s*2.f*sqrt2d3, xcnt++)	
		{
			for(y=config->ymax*0.475f,ycnt=0;y<config->ymax*0.98f-1.4f*r0;y+=r0s*sqrt3,ycnt++)//ok
			{
				for(z= config->zmax*0.6f*125.f/(125.f+35.f+10.f)+(r0s)*(ycnt%2==1)+(r0s*2.f/sqrt3)*(xcnt%2==1),zcnt=0;z<config->zmax*1.0f*125.f/(125.f+35.f+10.f) - 1.0f*r0;z+=2*r0s,zcnt++) //ok				
				{
					within_body_shell = -1;
				
					if( (ycnt== 0)&&(zcnt>= 19)&&(zcnt<= 32) ) within_body_shell=1;			// 1
					if( (ycnt== 1)&&(zcnt>= 15)&&(zcnt<= 36+4) ) within_body_shell=1;		// 2
					if( (ycnt== 2)&&(zcnt>= 13)&&(zcnt<= 40+7) ) within_body_shell=1;		// 3
					if( (ycnt== 3)&&(zcnt>= 10+1)&&(zcnt<= 44+17) ) within_body_shell=1;	// 4
					if( (ycnt== 4)&&(zcnt>=  7+3)&&(zcnt<= 68-4) ) within_body_shell=1;		// 5
					if( (ycnt== 5)&&(zcnt>=  4+3)&&(zcnt<= 69-3) ) within_body_shell=1;		// 6*
					if( (ycnt== 6)&&(zcnt>=  3+3)&&(zcnt<= 71-1) ) within_body_shell=1;		// 7*
					if( (ycnt== 7)&&(zcnt>=  2+2)&&(zcnt<= 78+3+2-1) ) within_body_shell=1;	// 8*
					if( (ycnt== 8)&&(zcnt>=  3+1)&&(zcnt<= 84+4) ) within_body_shell=1;		// 9*
					if( (ycnt== 9)&&(zcnt>=  2)&&(zcnt<= 89+2+1) ) within_body_shell=1;		//10
					if( (ycnt==10)&&(zcnt>=  2+1)&&(zcnt<= 93+1+1+1) ) within_body_shell=1;
					if( (ycnt==11)&&(zcnt>=  2)&&(zcnt<= 95+1+1) ) within_body_shell=1;
					if( (ycnt==12)&&(zcnt>=  2)&&(zcnt<= 97+1+1) ) within_body_shell=1;
					if( (ycnt==13)&&(zcnt>=  1)&&(zcnt<= 98+1+1) ) within_body_shell=1;//*
					if( (ycnt==14)&&(zcnt>=  1)&&(zcnt<= 99+1+1) ) within_body_shell=1;//*
					if( (ycnt==15)&&(zcnt>=  0)&&(zcnt<= 99+1+1) ) within_body_shell=1;//*
					if( (ycnt==16)&&(zcnt>=  1)&&(zcnt<=100+1+1) ) within_body_shell=1;//*
					if( (ycnt==17)&&(zcnt>=  0)&&(zcnt<=100+2+1) ) within_body_shell=1;
					if( (ycnt==18)&&(zcnt>=  1)&&(zcnt<=101+1) ) within_body_shell=1;//*
					if( (ycnt==19)&&(zcnt>=  1)&&(zcnt<=100+1) ) within_body_shell=1;//*
					if( (ycnt==20)&&(zcnt>=  2)&&(zcnt<=100+1) ) within_body_shell=1;//*
					if( (ycnt==21)&&(zcnt>=  2)&&(zcnt<= 99+1) ) within_body_shell=1;//*
					if( (ycnt==22)&&(zcnt>=  4)&&(zcnt<= 99+1) ) within_body_shell=1;
					if( (ycnt==23)&&(zcnt>=  6)&&(zcnt<= 97+1) ) within_body_shell=1;
					if( (ycnt==24)&&(zcnt>=  9)&&(zcnt<= 95+1+1) ) within_body_shell=1;
					if( (ycnt==25)&&(zcnt>= 16)&&(zcnt<= 92+2) ) within_body_shell=1;
					if( (ycnt==26)&&(zcnt>= 22)&&(zcnt<= 87+5) ) within_body_shell=1;
					if( (ycnt==27)&&(zcnt>= 42-15)&&(zcnt<= 79+8) ) within_body_shell=1;
					if( (ycnt==28)&&(zcnt>= 42-2)&&(zcnt<= 79+2) ) within_body_shell=1;
					
					if((xcnt>=11/*10*/)&&(xcnt<=11/*12*/))
					{
						if(within_body_shell!=1) within_body_shell=-1;
					}
					else
					if(within_body_shell==1) //simple tadpole
					{
						within_body_shell= -1;
						xx = xcnt;
						yy = ycnt;
						zz = zcnt+0.5f*(ycnt%2==1);
						
						if( (xx-11)*(xx-11)*0.59f*1.0 + (yy-16)*(yy-16)      + (z-262)*(z-262)*0.4f  < 52 ) within_body_shell=1;//forehead
						if( (xx-11)*(xx-11)*0.62f*1.0 + (yy-16)*(yy-16)      + (z-265)*(z-265)*0.4f  < 52 ) within_body_shell=1;//head
						if( (xx-11)*(xx-11)*0.67f*1.0 + (yy-13)*(yy-13)      + (z-267)*(z-267)*0.4f  < 52 ) within_body_shell=1;//head (low)
						if( (xx-11)*(xx-11)*0.69f*1.0 + (yy-16)*(yy-16)      + (z-269)*(z-269)*0.4f  < 52 ) within_body_shell=1;//head-neck
						if( (xx-11)*(xx-11)*1.10f*1.0 + (yy-16)*(yy-16)      + (z-273)*(z-273)*0.4f  < 52 ) within_body_shell=1;//head-neck
						if( (xx-11)*(xx-11)*1.40f*1.5 + (yy- 9)*(yy- 9)*4    + (z-261)*(z-261)*2.0f  < 52 ) within_body_shell=1;//mouth
						
						if(yy<14)
						{
						if( (xx-11)*(xx-11)*0.95f + (yy- 8)*(yy- 8)*0.78f + (z-289)*(z-289)*0.26f < 52 ) within_body_shell=1;//pre-low belly?
						if( (xx-11)*(xx-11)*1.35f + (yy- 9)*(yy- 9)*0.60f + (z-300)*(z-300)*0.038f < 52 ) within_body_shell=1;//middle belly large
						if( (xx-11)*(xx-11)*1.45f + (yy-18)*(yy-18)*1.7f + (z-310)*(z-310)*0.018f < 52 ) within_body_shell=1;//near front notochord (neck)
						if( (xx-11)*(xx-11)*1.45f + (yy-20)*(yy-20)*1.7f + (z-308)*(z-308)*0.018f < 52 ) within_body_shell=1;//near front notochord2 (neck)
						if( (xx-11)*(xx-11)*2.20f + (yy-16)*(yy-16)*0.25f + (z-319)*(z-319)*0.033f < 52 ) within_body_shell=1;//posterior belly
						if( (xx-11)*(xx-11)*2.20f + (yy-19)*(yy-19)*0.25f + (z-325)*(z-325)*0.033f < 52 ) within_body_shell=1;//posterior belly
						}
						//if( (xx-11)*(xx-11)*2.50f + (yy-15)*(yy-15)*1.7f + (z-330)*(z-330)*0.035f < 52 ) within_body_shell=1;//posterior belly2
					//	if( (xx-11)*(xx-11)*2.50f + (yy-18)*(yy-18)*1.7f + (z-330)*(z-330)*0.035f < 52 ) within_body_shell=1;//front part of muscles3
						//if( (xx-11)*(xx-11)*2.50f + (yy-21)*(yy-21)*1.7f + (z-330)*(z-330)*0.035f < 52 ) within_body_shell=1;//front part of muscles4
					//	if( (xx-11)*(xx-11)*3.00f + (yy-18)*(yy-18)*1.5f + (z-345)*(z-345)*0.010f < 52 ) within_body_shell=1;//front part of muscles4

						
						if((yy>=14))
						{
							if((xcnt== 10)||(xcnt==12))
							{
								if((yy==22)&&(zz>15)&&(zz<32)) within_body_shell=1;
								if((yy==21)&&(zz>15)&&(zz<55)) within_body_shell=1;
								if((yy==20)&&(zz>15)&&(zz<76)) within_body_shell=1;
								if((yy==19)&&(zz>15)&&(zz<92)) within_body_shell=1;
								if((yy==18)&&(zz>15)&&(zz<100)) within_body_shell=1;
								if((yy==17)&&(zz>15)&&(zz<100)) within_body_shell=1;
								if((yy==16)&&(zz>15)&&(zz<97)) within_body_shell=1;
								if((yy==15)&&(zz>15)&&(zz<88)) within_body_shell=1;
								if((yy==14)&&(zz>15)&&(zz<70)) within_body_shell=1;
							}

							if((xcnt== 9)||(xcnt==13))
							{
								if((yy==22)&&(zz>15)&&(zz<31)) within_body_shell=1;
								if((yy==21)&&(zz>15)&&(zz<55-1)) within_body_shell=1;
								if((yy==20)&&(zz>15)&&(zz<76-1+1)) within_body_shell=1;
								if((yy==19)&&(zz>15)&&(zz<81)) within_body_shell=1;
								if((yy==18)&&(zz>15)&&(zz<81+5)) within_body_shell=1;
								if((yy==17)&&(zz>15)&&(zz<80+6)) within_body_shell=1;
								if((yy==16)&&(zz>15)&&(zz<81+5)) within_body_shell=1;
								if((yy==15)&&(zz>15)&&(zz<81)) within_body_shell=1;
								if((yy==14)&&(zz>15)&&(zz<70)) within_body_shell=1;
							}

							if((xcnt== 8)||(xcnt==14))
							{
								if((yy==22)&&(zz>15)&&(zz<31)) within_body_shell=1;
								if((yy==21)&&(zz>15)&&(zz<55-1)) within_body_shell=1;
								if((yy==20)&&(zz>15)&&(zz<68)) within_body_shell=1;
								if((yy==19)&&(zz>15)&&(zz<67)) within_body_shell=1;
								if((yy==18)&&(zz>15)&&(zz<67)) within_body_shell=1;
								if((yy==17)&&(zz>15)&&(zz<66)) within_body_shell=1;
								if((yy==16)&&(zz>15)&&(zz<67)) within_body_shell=1;
								if((yy==15)&&(zz>15)&&(zz<67)) within_body_shell=1;
								if((yy==14)&&(zz>15)&&(zz<68)) within_body_shell=1;
							}

							if((xcnt== 7)||(xcnt==15))
							{
								//if((yy==22)&&(zz>15)&&(zz<31)) within_body_shell=1;
								if((yy==21)&&(zz>15)&&(zz<42-1)) within_body_shell=1;
								if((yy==20)&&(zz>15)&&(zz<42-1)) within_body_shell=1;
								if((yy==19)&&(zz>15)&&(zz<41-1)) within_body_shell=1;
								if((yy==18)&&(zz>15)&&(zz<41-1)) within_body_shell=1;
								if((yy==17)&&(zz>15)&&(zz<40-1)) within_body_shell=1;
								if((yy==16)&&(zz>15)&&(zz<41-1)) within_body_shell=1;
								if((yy==15)&&(zz>15)&&(zz<41-1)) within_body_shell=1;
								if((yy==14)&&(zz>15)&&(zz<42-1)) within_body_shell=1;
							}

							

							if(within_body_shell!=1)
							{
								//if(y+0.05*z>85.5+0*2.f)
								if(yy>18)
								{
									if( (xx-11)*(xx-11)*2.80f + (yy-17)*(yy-17)*1.4f + (z-260)*(z-260)*0.0025f < 52 ) within_body_shell=2;//front part of muscles4
									if( (xx-11)*(xx-11)*3.00f + (yy-17)*(yy-17)*1.4f + (z-257)*(z-257)*0.0022f < 52 ) within_body_shell=2;//front part of muscles4
									if( (xx-11)*(xx-11)*2.70f + (yy-19)*(yy-19)*1.4f + (z-243)*(z-243)*0.0025f < 52 ) within_body_shell=2;//front part of muscles4
								}
							}
						}

						if(within_body_shell!=1)
						{
							if((xcnt== 10)||(xcnt==12))
							{
								if((ycnt==19)&&(zcnt==96)) within_body_shell=4;
								if((ycnt==20)&&(zcnt==97)) within_body_shell=4;
								if((ycnt==21)&&(zcnt==97)) within_body_shell=4;
								if((ycnt==22)&&(zcnt==98)) within_body_shell=4;
								///////////////
								if((ycnt==15)&&(zcnt==96)) within_body_shell=4;
								if((ycnt==14)&&(zcnt==97)) within_body_shell=4;
								if((ycnt==13)&&(zcnt==97)) within_body_shell=4;
								if((ycnt==12)&&(zcnt==98)) within_body_shell=4;

								if((ycnt==19)&&(zcnt==96-9*1)) within_body_shell=4;
								if((ycnt==20)&&(zcnt==97-9*1)) within_body_shell=4;
								if((ycnt==21)&&(zcnt==97-9*1)) within_body_shell=4;
								if((ycnt==22)&&(zcnt==98-9*1)) within_body_shell=4;
								if((ycnt==23)&&(zcnt==98-9*1)) within_body_shell=4;
								if((ycnt==24)&&(zcnt==99-9*1)) within_body_shell=4;
								if((ycnt==25)&&(zcnt==99-9*1)) within_body_shell=4;
								///////////////
								if((ycnt==15)&&(zcnt==96-9*1)) within_body_shell=4;
								if((ycnt==14)&&(zcnt==97-9*1)) within_body_shell=4;
								if((ycnt==13)&&(zcnt==97-9*1)) within_body_shell=4;
								if((ycnt==12)&&(zcnt==98-9*1)) within_body_shell=4;
								if((ycnt==11)&&(zcnt==98-9*1)) within_body_shell=4;
								if((ycnt==10)&&(zcnt==99-9*1)) within_body_shell=4;
								//if((ycnt== 9)&&(zcnt==99-9*1)) within_body_shell=2;

								if((ycnt==19)&&(zcnt==96-9*2)) within_body_shell=4;
								if((ycnt==20)&&(zcnt==97-9*2)) within_body_shell=4;
								if((ycnt==21)&&(zcnt==97-9*2)) within_body_shell=4;
								if((ycnt==22)&&(zcnt==98-9*2)) within_body_shell=4;
								if((ycnt==23)&&(zcnt==98-9*2)) within_body_shell=4;
								if((ycnt==24)&&(zcnt==99-9*2)) within_body_shell=4;
								if((ycnt==25)&&(zcnt==99-9*2)) within_body_shell=4;
								if((ycnt==26)&&(zcnt==100-9*2)) within_body_shell=4;
								///////////////
								if((ycnt==15)&&(zcnt==96-9*2)) within_body_shell=4;
								if((ycnt==14)&&(zcnt==97-9*2)) within_body_shell=4;
								if((ycnt==13)&&(zcnt==97-9*2)) within_body_shell=4;
								if((ycnt==12)&&(zcnt==98-9*2)) within_body_shell=4;
								if((ycnt==11)&&(zcnt==98-9*2)) within_body_shell=4;
								if((ycnt==10)&&(zcnt==99-9*2)) within_body_shell=4;
								if((ycnt== 9)&&(zcnt==99-9*2)) within_body_shell=4;
								if((ycnt== 8)&&(zcnt==100-9*2)) within_body_shell=4;

								if((ycnt==19)&&(zcnt==96-9*3)) within_body_shell=4;
								if((ycnt==20)&&(zcnt==97-9*3)) within_body_shell=4;
								if((ycnt==21)&&(zcnt==97-9*3)) within_body_shell=4;
								if((ycnt==22)&&(zcnt==98-9*3)) within_body_shell=4;
								if((ycnt==23)&&(zcnt==98-9*3)) within_body_shell=4;
								if((ycnt==24)&&(zcnt==99-9*3)) within_body_shell=4;
								if((ycnt==25)&&(zcnt==99-9*3)) within_body_shell=4;
								if((ycnt==26)&&(zcnt==100-9*3)) within_body_shell=4;
								if((ycnt==27)&&(zcnt==100-9*3)) within_body_shell=4;
								///////////////
								if((ycnt==15)&&(zcnt==96-9*3)) within_body_shell=4;
								if((ycnt==14)&&(zcnt==97-9*3)) within_body_shell=4;
								if((ycnt==13)&&(zcnt==97-9*3)) within_body_shell=4;
								if((ycnt==12)&&(zcnt==98-9*3)) within_body_shell=4;
								if((ycnt==11)&&(zcnt==98-9*3)) within_body_shell=4;
								if((ycnt==10)&&(zcnt==99-9*3)) within_body_shell=4;
								if((ycnt== 9)&&(zcnt==99-9*3)) within_body_shell=4;
								if((ycnt== 8)&&(zcnt==100-9*3)) within_body_shell=4;

								if((ycnt==19)&&(zcnt==96-9*4)) within_body_shell=4;
								if((ycnt==20)&&(zcnt==97-9*4)) within_body_shell=4;
								if((ycnt==21)&&(zcnt==97-9*4)) within_body_shell=4;
								if((ycnt==22)&&(zcnt==98-9*4)) within_body_shell=4;
								if((ycnt==23)&&(zcnt==98-9*4)) within_body_shell=4;
								if((ycnt==24)&&(zcnt==99-9*4)) within_body_shell=4;
								if((ycnt==25)&&(zcnt==99-9*4)) within_body_shell=4;
								if((ycnt==26)&&(zcnt==100-9*4)) within_body_shell=4;
								if((ycnt==27)&&(zcnt==100-9*4)) within_body_shell=4;
								///////////////
								if((ycnt==15)&&(zcnt==96-9*4)) within_body_shell=4;
								if((ycnt==14)&&(zcnt==97-9*4)) within_body_shell=4;
								if((ycnt==13)&&(zcnt==97-9*4)) within_body_shell=4;
								if((ycnt==12)&&(zcnt==98-9*4)) within_body_shell=4;
								if((ycnt==11)&&(zcnt==98-9*4)) within_body_shell=4;
								if((ycnt==10)&&(zcnt==99-9*4)) within_body_shell=4;
								if((ycnt== 9)&&(zcnt==99-9*4)) within_body_shell=4;
								if((ycnt== 8)&&(zcnt==100-9*4)) within_body_shell=4;
								if((ycnt== 7)&&(zcnt==100-9*4)) within_body_shell=4;
								if((ycnt== 6)&&(zcnt==101-9*4)) within_body_shell=4;
								//if((ycnt== 5)&&(zcnt==101-9*4)) within_body_shell=2;

								if((ycnt==19)&&(zcnt==96-9*5)) within_body_shell=4;
								if((ycnt==20)&&(zcnt==97-9*5)) within_body_shell=4;
								if((ycnt==21)&&(zcnt==97-9*5)) within_body_shell=4;
								if((ycnt==22)&&(zcnt==98-9*5)) within_body_shell=4;
								if((ycnt==23)&&(zcnt==98-9*5)) within_body_shell=4;
								if((ycnt==24)&&(zcnt==99-9*5)) within_body_shell=4;
								if((ycnt==25)&&(zcnt==99-9*5)) within_body_shell=4;
								if((ycnt==26)&&(zcnt==100-9*5)) within_body_shell=4;
								if((ycnt==27)&&(zcnt==100-9*5)) within_body_shell=4;

								if((ycnt==19)&&(zcnt==96-9*6)) within_body_shell=4;
								if((ycnt==20)&&(zcnt==97-9*6)) within_body_shell=4;
								if((ycnt==21)&&(zcnt==97-9*6)) within_body_shell=4;
								if((ycnt==22)&&(zcnt==98-9*6)) within_body_shell=4;
								if((ycnt==23)&&(zcnt==98-9*6)) within_body_shell=4;
								if((ycnt==24)&&(zcnt==99-9*6)) within_body_shell=4;
								if((ycnt==25)&&(zcnt==99-9*6)) within_body_shell=4;
								if((ycnt==26)&&(zcnt==100-9*6)) within_body_shell=4;
								if((ycnt==27)&&(zcnt==100-9*6)) within_body_shell=4;

								if((ycnt==24)&&(zcnt==99-9*7)) within_body_shell=4;
								if((ycnt==25)&&(zcnt==99-9*7)) within_body_shell=4;
								if((ycnt==26)&&(zcnt==100-9*7)) within_body_shell=4;
								if((ycnt==27)&&(zcnt==100-9*7)) within_body_shell=4;

								if((ycnt==24)&&(zcnt==99-9*8)) within_body_shell=4;
								if((ycnt==25)&&(zcnt==99-9*8)) within_body_shell=4;
								if((ycnt==26)&&(zcnt==100-9*8)) within_body_shell=4;
								if((ycnt==27)&&(zcnt==100-9*8)) within_body_shell=4;

								//tail
								if((ycnt==17)&&(zcnt==100)) within_body_shell=4;
								if((ycnt==17)&&(zcnt==101)) within_body_shell=4;
								if((ycnt==17)&&(zcnt==102)) within_body_shell=4;
							}
						}

						/*
						if( (xx-11)*(xx-11)*15.f + (yy-17)*(yy-17)*0.27f + (z-285)*(z-285)*12.5f < 52 ) within_body_shell=1;//
						if( (xx-11)*(xx-11)*15.f + (yy-17)*(yy-17)*0.27f + (z-300)*(z-300)*12.5f < 52 ) within_body_shell=1;//
						if( (xx-11)*(xx-11)*15.f + (yy-17)*(yy-17)*0.27f + (z-315)*(z-315)*12.5f < 52 ) within_body_shell=1;//
						if( (xx-11)*(xx-11)*15.f + (yy-17)*(yy-17)*0.27f + (z-330)*(z-330)*12.5f < 52 ) within_body_shell=1;//
						if( (xx-11)*(xx-11)*15.f + (yy-17)*(yy-17)*0.27f + (z-345)*(z-345)*12.5f < 52 ) within_body_shell=1;//
						if( (xx-11)*(xx-11)*15.f + (yy-17)*(yy-17)*0.27f + (z-360)*(z-360)*12.5f < 52 ) within_body_shell=1;//
						if( (xx-11)*(xx-11)*15.f + (yy-17)*(yy-17)*0.27f + (z-375)*(z-375)*12.5f < 52 ) within_body_shell=1;//
						if( (xx-11)*(xx-11)*15.f + (yy-17)*(yy-17)*0.27f + (z-390)*(z-390)*12.5f < 52 ) within_body_shell=1;//
						if( (xx-11)*(xx-11)*15.f + (yy-17)*(yy-17)*0.27f + (z-405)*(z-405)*12.5f < 52 ) within_body_shell=1;//*/

					}
					
					// tadpole 2.3
					if(within_body_shell>=0)
					{
						if(stage==1)
						{	
							positionVector = position_cpp + 4 * (pCount+i_start);
							positionVector[ 0 ] = x;
							positionVector[ 1 ] = y;
							positionVector[ 2 ] = z;// + (r0s*2.f/sqrt3)*(xcnt%2==1);
							positionVector[ 3 ] = 2.4f;
							//if(xcnt%2==0) positionVector[ 3 ] = 2.3f;
							//if(ycnt%2==0) positionVector[ 3 ] = 2.3f;


							if(within_body_shell>0) 
							{
								positionVector[ 3 ] = 2.4f;

								if(within_body_shell==1) 
								{
								positionVector[ 3 ] = 2.4f;//ordinary body particles

								if((zz>18)&&(yy<14.0))
								{
								if( (xx-11)*(xx-11)*0.60f + (yy-8.5+1)*(yy-8.5-5.5)*0.6f + (z-295-10)*(z-295-10)*0.020f < 52 )  positionVector[ 3 ] = 2.32f; //belly
								if( (xx-11)*(xx-11)*0.70f + (yy-8.5+1)*(yy-8.5+1)*1.0f + (z-291-10)*(z-291-10)*0.045f < 52 )  positionVector[ 3 ] = 2.32f;
								if( (xx-11)*(xx-11)*0.55f + (yy-8.0+1)*(yy-8.0+1)*0.95f + (z-288-10)*(z-288-10)*0.0450f < 52 )  positionVector[ 3 ] = 2.32f;
								}

								//if(  (ycnt-15)*(ycnt-15)*0.45f + (z-262)*(z-262)*0.35 < 90 )  positionVector[ 3 ] = 2.4f; // exclude head

								if( (xcnt- 1)*(xcnt- 1)*0.5 + (ycnt-18.5)*(ycnt-18.5)*0.90f + (z-262.2)*(z-262.2)*0.5 < 10 )  positionVector[ 3 ] = 2.3f;//eyes
								if( (xcnt-21)*(xcnt-21)*0.5 + (ycnt-18.5)*(ycnt-18.5)*0.90f + (z-262.2)*(z-262.2)*0.5 < 10 )  positionVector[ 3 ] = 2.3f;//eyes

								if( (xcnt- 11)*(xcnt- 11)*0.3 + (ycnt- 5)*(ycnt- 5)*0.35f + (z-256)*(z-256)*0.30 < 10 )  positionVector[ 3 ] = 2.31f;//mouth

								z_shift=0;

								if((xcnt==7)||(xcnt==15)) zcnt += +1;
								if((xcnt==10)||(xcnt==12)) zcnt += -1;

								//if(y+0.05*z<85.5)
								if((xcnt!=11)/*&&(positionVector[3]>2.39)*/)
								{
									
									if((xcnt== 8)||(xcnt==14)) {
									if((ycnt==19)&&(zcnt>=12)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f;// 0 - ordinary springs
									if((ycnt==18)&&(zcnt>= 9)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f;// 1 and higher values - muscle fibers
									if((ycnt==17)&&(zcnt>= 6+6)&&(zcnt<=15)) positionVector[ 3 ] += 0.00001f;
									if((ycnt==16)&&(zcnt>= 6+6)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f; }

									if((xcnt== 9)||(xcnt==13)) {
									if((ycnt==19)&&(zcnt>=12)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f;
									if((ycnt==18)&&(zcnt>= 9)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f;
									if((ycnt==17)&&(zcnt>= 6+3)&&(zcnt<=15)) positionVector[ 3 ] += 0.00001f;
									if((ycnt==16)&&(zcnt>= 6+3)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f; }
									
									if((xcnt== 7)||(xcnt==15)) {
									if((ycnt==19)&&(zcnt>=  12+2)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f;
									if((ycnt==18)&&(zcnt>= 9+3+2)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f;
									if((ycnt==17)&&(zcnt>= 6+3+5)&&(zcnt<=15)) positionVector[ 3 ] += 0.00001f;
									if((ycnt==16)&&(zcnt>= 6+3+5)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f; }

									if((xcnt==10)||(xcnt==12)) {
									if((ycnt==19)&&(zcnt>=12)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f;
									if((ycnt==18)&&(zcnt>= 9)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f;
									if((ycnt==17)&&(zcnt>= 6)&&(zcnt<=15)) positionVector[ 3 ] += 0.00001f;
									if((ycnt==16)&&(zcnt>= 6)&&(zcnt<=16)) positionVector[ 3 ] += 0.00001f; }
									
									
									
									if( ((xcnt>= 7)&&(xcnt<= 9+1))||((xcnt>=13-1)&&(xcnt<=15))) {//////////////
										
									if((ycnt==22)&&(zcnt>=21)&&(zcnt<=22)) positionVector[ 3 ] += 0.00002f;
									if((ycnt==21)&&(zcnt>=20)&&(zcnt<=21)) positionVector[ 3 ] += 0.00002f;
									if((ycnt==20)&&(zcnt>=19)&&(zcnt<=21)) positionVector[ 3 ] += 0.00002f;
									if((ycnt==19)&&(zcnt>=17)&&(zcnt<=20)) positionVector[ 3 ] += 0.00002f;
									if((ycnt==18)&&(zcnt>=17)&&(zcnt<=20)) positionVector[ 3 ] += 0.00002f;
									if((ycnt==17)&&(zcnt>=16)&&(zcnt<=19)) positionVector[ 3 ] += 0.00002f;
									if((ycnt==16)&&(zcnt>=17)&&(zcnt<=20)) positionVector[ 3 ] += 0.00002f;
									//if((ycnt==15)&&(zcnt>=17)&&(zcnt<=19)) positionVector[ 3 ] += 0.00002f;
									
									if((ycnt==22)&&(zcnt>=23)&&(zcnt<=25)) positionVector[ 3 ] += 0.00003f;
									if((ycnt==21)&&(zcnt>=22)&&(zcnt<=24)) positionVector[ 3 ] += 0.00003f;
									if((ycnt==20)&&(zcnt>=22)&&(zcnt<=24)) positionVector[ 3 ] += 0.00003f;
									if((ycnt==19)&&(zcnt>=21)&&(zcnt<=23)) positionVector[ 3 ] += 0.00003f;
									if((ycnt==18)&&(zcnt>=21)&&(zcnt<=23)) positionVector[ 3 ] += 0.00003f;
									if((ycnt==17)&&(zcnt>=20)&&(zcnt<=22)) positionVector[ 3 ] += 0.00003f;
									if((ycnt==16)&&(zcnt>=21)&&(zcnt<=23)) positionVector[ 3 ] += 0.00003f;
									//if((ycnt==15)&&(zcnt>=23)&&(zcnt<=23)) positionVector[ 3 ] += 0.00003f; 

									if((ycnt==22)&&(zcnt>=26)&&(zcnt<=28)) positionVector[ 3 ] += 0.00004f;
									if((ycnt==21)&&(zcnt>=25)&&(zcnt<=27)) positionVector[ 3 ] += 0.00004f;
									if((ycnt==20)&&(zcnt>=25)&&(zcnt<=27)) positionVector[ 3 ] += 0.00004f;
									if((ycnt==19)&&(zcnt>=24)&&(zcnt<=26)) positionVector[ 3 ] += 0.00004f;
									if((ycnt==18)&&(zcnt>=24)&&(zcnt<=26)) positionVector[ 3 ] += 0.00004f;
									if((ycnt==17)&&(zcnt>=23)&&(zcnt<=25)) positionVector[ 3 ] += 0.00004f;
									if((ycnt==16)&&(zcnt>=24)&&(zcnt<=26)) positionVector[ 3 ] += 0.00004f;
									if((ycnt==15)&&(zcnt>=24)&&(zcnt<=26)) positionVector[ 3 ] += 0.00004f; 
									
									if((ycnt==22)&&(zcnt>=29)&&(zcnt<=31)) positionVector[ 3 ] += 0.00005f;
									if((ycnt==21)&&(zcnt>=28)&&(zcnt<=30)) positionVector[ 3 ] += 0.00005f;
									if((ycnt==20)&&(zcnt>=28)&&(zcnt<=30)) positionVector[ 3 ] += 0.00005f;
									if((ycnt==19)&&(zcnt>=27)&&(zcnt<=29)) positionVector[ 3 ] += 0.00005f;
									if((ycnt==18)&&(zcnt>=27)&&(zcnt<=29)) positionVector[ 3 ] += 0.00005f;
									if((ycnt==17)&&(zcnt>=26)&&(zcnt<=28)) positionVector[ 3 ] += 0.00005f;
									if((ycnt==16)&&(zcnt>=27)&&(zcnt<=29)) positionVector[ 3 ] += 0.00005f;
									if((ycnt==15)&&(zcnt>=27)&&(zcnt<=29)) positionVector[ 3 ] += 0.00005f; 
									if((ycnt==14)&&(zcnt>=28)&&(zcnt<=30)) positionVector[ 3 ] += 0.00005f; 
									
									if((ycnt==22)&&(zcnt>=32)&&(zcnt<=34)) positionVector[ 3 ] += 0.00006f;
									if((ycnt==21)&&(zcnt>=31)&&(zcnt<=33)) positionVector[ 3 ] += 0.00006f;
									if((ycnt==20)&&(zcnt>=31)&&(zcnt<=33)) positionVector[ 3 ] += 0.00006f;
									if((ycnt==19)&&(zcnt>=30)&&(zcnt<=32)) positionVector[ 3 ] += 0.00006f;
									if((ycnt==18)&&(zcnt>=30)&&(zcnt<=32)) positionVector[ 3 ] += 0.00006f;
									if((ycnt==17)&&(zcnt>=29)&&(zcnt<=31)) positionVector[ 3 ] += 0.00006f;
									if((ycnt==16)&&(zcnt>=30)&&(zcnt<=32)) positionVector[ 3 ] += 0.00006f;
									if((ycnt==15)&&(zcnt>=30)&&(zcnt<=32)) positionVector[ 3 ] += 0.00006f; 
									if((ycnt==14)&&(zcnt>=31)&&(zcnt<=33)) positionVector[ 3 ] += 0.00006f; 
									
									if((ycnt==22)&&(zcnt>=35)&&(zcnt<=37)) positionVector[ 3 ] += 0.00007f;
									if((ycnt==21)&&(zcnt>=34)&&(zcnt<=36)) positionVector[ 3 ] += 0.00007f;
									if((ycnt==20)&&(zcnt>=34)&&(zcnt<=36)) positionVector[ 3 ] += 0.00007f;
									if((ycnt==19)&&(zcnt>=33)&&(zcnt<=35)) positionVector[ 3 ] += 0.00007f;
									if((ycnt==18)&&(zcnt>=33)&&(zcnt<=35)) positionVector[ 3 ] += 0.00007f;
									if((ycnt==17)&&(zcnt>=32)&&(zcnt<=34)) positionVector[ 3 ] += 0.00007f;
									if((ycnt==16)&&(zcnt>=33)&&(zcnt<=35)) positionVector[ 3 ] += 0.00007f;
									if((ycnt==15)&&(zcnt>=33)&&(zcnt<=35)) positionVector[ 3 ] += 0.00007f; 
									if((ycnt==14)&&(zcnt>=34)&&(zcnt<=36)) positionVector[ 3 ] += 0.00007f; 
									
									if((ycnt==22)&&(zcnt>=38)&&(zcnt<=40)) positionVector[ 3 ] += 0.00008f;
									if((ycnt==21)&&(zcnt>=37)&&(zcnt<=39)) positionVector[ 3 ] += 0.00008f;
									if((ycnt==20)&&(zcnt>=37)&&(zcnt<=39)) positionVector[ 3 ] += 0.00008f;
									if((ycnt==19)&&(zcnt>=36)&&(zcnt<=38)) positionVector[ 3 ] += 0.00008f;
									if((ycnt==18)&&(zcnt>=36)&&(zcnt<=38)) positionVector[ 3 ] += 0.00008f;
									if((ycnt==17)&&(zcnt>=35)&&(zcnt<=37)) positionVector[ 3 ] += 0.00008f;
									if((ycnt==16)&&(zcnt>=36)&&(zcnt<=38)) positionVector[ 3 ] += 0.00008f;
									if((ycnt==15)&&(zcnt>=36)&&(zcnt<=38)) positionVector[ 3 ] += 0.00008f; 
									if((ycnt==14)&&(zcnt>=37)&&(zcnt<=39)) positionVector[ 3 ] += 0.00008f; 
									
									if((ycnt==22)&&(zcnt>=41)&&(zcnt<=43)) positionVector[ 3 ] += 0.00009f;
									if((ycnt==21)&&(zcnt>=40)&&(zcnt<=42)) positionVector[ 3 ] += 0.00009f;
									if((ycnt==20)&&(zcnt>=40)&&(zcnt<=42)) positionVector[ 3 ] += 0.00009f;
									if((ycnt==19)&&(zcnt>=39)&&(zcnt<=41)) positionVector[ 3 ] += 0.00009f;
									if((ycnt==18)&&(zcnt>=39)&&(zcnt<=41)) positionVector[ 3 ] += 0.00009f;
									if((ycnt==17)&&(zcnt>=38)&&(zcnt<=40)) positionVector[ 3 ] += 0.00009f;
									if((ycnt==16)&&(zcnt>=39)&&(zcnt<=41)) positionVector[ 3 ] += 0.00009f;
									if((ycnt==15)&&(zcnt>=39)&&(zcnt<=41)) positionVector[ 3 ] += 0.00009f; 
									if((ycnt==14)&&(zcnt>=40)&&(zcnt<=42)) positionVector[ 3 ] += 0.00009f; 

									if( ((ycnt==22)&&(zcnt>=44)&&(zcnt<=46)) ||
									    ((ycnt==21)&&(zcnt>=43)&&(zcnt<=45)) ||
										((ycnt==20)&&(zcnt>=43)&&(zcnt<=45)) ||
										((ycnt==19)&&(zcnt>=42)&&(zcnt<=44)) ||
										((ycnt==18)&&(zcnt>=42)&&(zcnt<=44)) ||
										((ycnt==17)&&(zcnt>=41)&&(zcnt<=43)) ||
										((ycnt==16)&&(zcnt>=42)&&(zcnt<=44)) ||
										((ycnt==15)&&(zcnt>=42)&&(zcnt<=44)) ||
										((ycnt==14)&&(zcnt>=43)&&(zcnt<=45))  ) positionVector[ 3 ] += 0.00010f; 
									
									if( ((ycnt==22)&&(zcnt>=47)&&(zcnt<=49)) ||
									    ((ycnt==21)&&(zcnt>=46)&&(zcnt<=48)) ||
										((ycnt==20)&&(zcnt>=46)&&(zcnt<=48)) ||
										((ycnt==19)&&(zcnt>=45)&&(zcnt<=47)) ||
										((ycnt==18)&&(zcnt>=45)&&(zcnt<=47)) ||
										((ycnt==17)&&(zcnt>=44)&&(zcnt<=46)) ||
										((ycnt==16)&&(zcnt>=45)&&(zcnt<=47)) ||
										((ycnt==15)&&(zcnt>=45)&&(zcnt<=47)) ||
										((ycnt==14)&&(zcnt>=46)&&(zcnt<=48))  ) positionVector[ 3 ] += 0.00011f; 

									if( ((ycnt==22)&&(zcnt>=50)&&(zcnt<=52)) ||
									    ((ycnt==21)&&(zcnt>=49)&&(zcnt<=51)) ||
										((ycnt==20)&&(zcnt>=49)&&(zcnt<=51)) ||
										((ycnt==19)&&(zcnt>=48)&&(zcnt<=50)) ||
										((ycnt==18)&&(zcnt>=48)&&(zcnt<=50)) ||
										((ycnt==17)&&(zcnt>=47)&&(zcnt<=49)) ||
										((ycnt==16)&&(zcnt>=48)&&(zcnt<=50)) ||
										((ycnt==15)&&(zcnt>=48)&&(zcnt<=50)) ||
										((ycnt==14)&&(zcnt>=49)&&(zcnt<=51))  ) positionVector[ 3 ] += 0.00012f; 

									if( ((ycnt==22)&&(zcnt>=53)&&(zcnt<=55)) ||
									    ((ycnt==21)&&(zcnt>=52)&&(zcnt<=54)) ||
										((ycnt==20)&&(zcnt>=52)&&(zcnt<=54)) ||
										((ycnt==19)&&(zcnt>=51)&&(zcnt<=53)) ||
										((ycnt==18)&&(zcnt>=51)&&(zcnt<=53)) ||
										((ycnt==17)&&(zcnt>=50)&&(zcnt<=52)) ||
										((ycnt==16)&&(zcnt>=51)&&(zcnt<=53)) ||
										((ycnt==15)&&(zcnt>=51)&&(zcnt<=53)) ||
										((ycnt==14)&&(zcnt>=52)&&(zcnt<=54))  ) positionVector[ 3 ] += 0.00013f; 

									if( ((ycnt==22)&&(zcnt>=56)&&(zcnt<=58)) ||
									    ((ycnt==21)&&(zcnt>=55)&&(zcnt<=57)) ||
										((ycnt==20)&&(zcnt>=55)&&(zcnt<=57)) ||
										((ycnt==19)&&(zcnt>=54)&&(zcnt<=56)) ||
										((ycnt==18)&&(zcnt>=54)&&(zcnt<=56)) ||
										((ycnt==17)&&(zcnt>=53)&&(zcnt<=55)) ||
										((ycnt==16)&&(zcnt>=54)&&(zcnt<=56)) ||
										((ycnt==15)&&(zcnt>=54)&&(zcnt<=56)) ||
										((ycnt==14)&&(zcnt>=55)&&(zcnt<=57))  ) positionVector[ 3 ] += 0.00014f; 

									if( ((ycnt==22)&&(zcnt>=59)&&(zcnt<=61)) ||
									    ((ycnt==21)&&(zcnt>=58)&&(zcnt<=60)) ||
										((ycnt==20)&&(zcnt>=58)&&(zcnt<=60)) ||
										((ycnt==19)&&(zcnt>=57)&&(zcnt<=59)) ||
										((ycnt==18)&&(zcnt>=57)&&(zcnt<=59)) ||
										((ycnt==17)&&(zcnt>=56)&&(zcnt<=58)) ||
										((ycnt==16)&&(zcnt>=57)&&(zcnt<=59)) ||
										((ycnt==15)&&(zcnt>=57)&&(zcnt<=59)) ||
										((ycnt==14)&&(zcnt>=58)&&(zcnt<=60))  ) positionVector[ 3 ] += 0.00015f; 
									
									if( ((ycnt==22)&&(zcnt>=62)&&(zcnt<=64)) ||
									    ((ycnt==21)&&(zcnt>=61)&&(zcnt<=63)) ||
										((ycnt==20)&&(zcnt>=61)&&(zcnt<=63)) ||
										((ycnt==19)&&(zcnt>=60)&&(zcnt<=62)) ||
										((ycnt==18)&&(zcnt>=60)&&(zcnt<=62)) ||
										((ycnt==17)&&(zcnt>=59)&&(zcnt<=61)) ||
										((ycnt==16)&&(zcnt>=60)&&(zcnt<=62)) ||
										((ycnt==15)&&(zcnt>=60)&&(zcnt<=62)) ||
										((ycnt==14)&&(zcnt>=61)&&(zcnt<=63))  ) positionVector[ 3 ] += 0.00016f; 

									if( ((ycnt==22)&&(zcnt>=65)&&(zcnt<=67)) ||
									    ((ycnt==21)&&(zcnt>=64)&&(zcnt<=66)) ||
										((ycnt==20)&&(zcnt>=64)&&(zcnt<=66)) ||
										((ycnt==19)&&(zcnt>=63)&&(zcnt<=65)) ||
										((ycnt==18)&&(zcnt>=63)&&(zcnt<=65)) ||
										((ycnt==17)&&(zcnt>=62)&&(zcnt<=64)) ||
										((ycnt==16)&&(zcnt>=63)&&(zcnt<=65)) ||
										((ycnt==15)&&(zcnt>=63)&&(zcnt<=65)) ||
										((ycnt==14)&&(zcnt>=64)&&(zcnt<=66))  ) positionVector[ 3 ] += 0.00017f; 

									if( ((ycnt==22)&&(zcnt>=68)&&(zcnt<=70-1)) ||
									    ((ycnt==21)&&(zcnt>=67)&&(zcnt<=69-1)) ||
										((ycnt==20)&&(zcnt>=67)&&(zcnt<=69-1)) ||
										((ycnt==19)&&(zcnt>=66)&&(zcnt<=68-1)) ||
										((ycnt==18)&&(zcnt>=66)&&(zcnt<=68-1)) ||
										((ycnt==17)&&(zcnt>=65)&&(zcnt<=67-1)) ||
										((ycnt==16)&&(zcnt>=66)&&(zcnt<=68-1)) ||
										((ycnt==15)&&(zcnt>=66)&&(zcnt<=68-1)) ||
										((ycnt==14)&&(zcnt>=67)&&(zcnt<=69-1))  ) positionVector[ 3 ] += 0.00018f; 
									
									if( ((ycnt==22)&&(zcnt>=71-1)&&(zcnt<=72-1)) ||
									    ((ycnt==21)&&(zcnt>=70-1)&&(zcnt<=71-1)) ||
										((ycnt==20)&&(zcnt>=70-1)&&(zcnt<=71-1)) ||
										((ycnt==19)&&(zcnt>=69-1)&&(zcnt<=70-1)) ||
										((ycnt==18)&&(zcnt>=69-1)&&(zcnt<=70-1)) ||
										((ycnt==17)&&(zcnt>=68-1)&&(zcnt<=69-1)) ||
										((ycnt==16)&&(zcnt>=69-1)&&(zcnt<=70-1)) ||
										((ycnt==15)&&(zcnt>=69-1)&&(zcnt<=70-1)) ||
										((ycnt==14)&&(zcnt>=70-1)&&(zcnt<=71-1))  ) positionVector[ 3 ] += 0.00019f; 
										
									if( ((ycnt==22)&&(zcnt>=73-1)&&(zcnt<=74-1)) ||
									    ((ycnt==21)&&(zcnt>=72-1)&&(zcnt<=73-1)) ||
										((ycnt==20)&&(zcnt>=72-1)&&(zcnt<=73-1)) ||
										((ycnt==19)&&(zcnt>=71-1)&&(zcnt<=72-1)) ||
										((ycnt==18)&&(zcnt>=71-1)&&(zcnt<=72-1)) ||
										((ycnt==17)&&(zcnt>=70-1)&&(zcnt<=71-1)) ||
										((ycnt==16)&&(zcnt>=71-1)&&(zcnt<=72-1)) ||
										((ycnt==15)&&(zcnt>=71-1)&&(zcnt<=72-1)) ||
										((ycnt==14)&&(zcnt>=72-1)&&(zcnt<=73-1))  ) positionVector[ 3 ] += 0.00020f; 
										
									if( ((ycnt==22)&&(zcnt>=73+2*1-1)&&(zcnt<=74+2*1-1)) ||
									    ((ycnt==21)&&(zcnt>=72+2*1-1)&&(zcnt<=73+2*1-1)) ||
										((ycnt==20)&&(zcnt>=72+2*1-1)&&(zcnt<=73+2*1-1)) ||
										((ycnt==19)&&(zcnt>=71+2*1-1)&&(zcnt<=72+2*1-1)) ||
										((ycnt==18)&&(zcnt>=71+2*1-1)&&(zcnt<=72+2*1-1)) ||
										((ycnt==17)&&(zcnt>=70+2*1-1)&&(zcnt<=71+2*1-1)) ||
										((ycnt==16)&&(zcnt>=71+2*1-1)&&(zcnt<=72+2*1-1)) ||
										((ycnt==15)&&(zcnt>=71+2*1-1)&&(zcnt<=72+2*1-1)) ||
										((ycnt==14)&&(zcnt>=72+2*1-1)&&(zcnt<=73+2*1-1))  ) positionVector[ 3 ] += 0.00021f; 
										
									if( ((ycnt==20)&&(zcnt>=72+2*2-1)&&(zcnt<=73+2*2-1)) ||
										((ycnt==19)&&(zcnt>=71+2*2-1)&&(zcnt<=72+2*2-1)) ||
										((ycnt==18)&&(zcnt>=71+2*2-1)&&(zcnt<=72+2*2-1)) ||
										((ycnt==17)&&(zcnt>=70+2*2-1)&&(zcnt<=71+2*2-1)) ||
										((ycnt==16)&&(zcnt>=71+2*2-1)&&(zcnt<=72+2*2-1)) ||
										((ycnt==15)&&(zcnt>=71+2*2-1)&&(zcnt<=72+2*2-1))  ) positionVector[ 3 ] += 0.00022f; 
										
									if( ((ycnt==19)&&(zcnt>=71+2*3-1)&&(zcnt<=72+2*3-1)) ||
										((ycnt==18)&&(zcnt>=71+2*3-1)&&(zcnt<=72+2*3-1)) ||
										((ycnt==17)&&(zcnt>=70+2*3-1)&&(zcnt<=71+2*3-1)) ||
										((ycnt==16)&&(zcnt>=71+2*3-1)&&(zcnt<=72+2*3-1)) ||
										((ycnt==15)&&(zcnt>=71+2*3-1)&&(zcnt<=72+2*3-1))  ) positionVector[ 3 ] += 0.00023f; 
										
									if( ((ycnt==19)&&(zcnt>=71+2*4-1)&&(zcnt<=72+2*4-1)) ||
										((ycnt==18)&&(zcnt>=71+2*4-1)&&(zcnt<=72+2*4-1)) ||
										((ycnt==17)&&(zcnt>=70+2*4-1)&&(zcnt<=71+2*4-1)) ||
										((ycnt==16)&&(zcnt>=71+2*4-1)&&(zcnt<=72+2*4-1)) ||
										((ycnt==15)&&(zcnt>=71+2*4-1)&&(zcnt<=72+2*4-1))  ) positionVector[ 3 ] += 0.00024f; 

									if( ((ycnt==19)&&(zcnt>=71+2*5-1)&&(zcnt<=72+2*5-1)) ||
										((ycnt==18)&&(zcnt>=71+2*5-1)&&(zcnt<=72+2*5-1)) ||
										((ycnt==17)&&(zcnt>=70+2*5-1)&&(zcnt<=71+2*5-1)) ||
										((ycnt==16)&&(zcnt>=71+2*5-1)&&(zcnt<=72+2*5-1)) ||
										((ycnt==15)&&(zcnt>=71+2*5-1)&&(zcnt<=72+2*5-1))  ) positionVector[ 3 ] += 0.00025f; 

									if( ((ycnt==19)&&(zcnt>=71+2*6-1)&&(zcnt<=72+2*6-1)) ||
										((ycnt==18)&&(zcnt>=71+2*6-1)&&(zcnt<=72+2*6-1)) ||
										((ycnt==17)&&(zcnt>=70+2*6-1)&&(zcnt<=71+2*6-1)) ||
										((ycnt==16)&&(zcnt>=71+2*6-1)&&(zcnt<=72+2*6-1)) ||
										((ycnt==15)&&(zcnt>=71+2*6-1)&&(zcnt<=72+2*6-1))  ) positionVector[ 3 ] += 0.00026f; 
										
									if( ((ycnt==19)&&(zcnt>=71+2*7-1)&&(zcnt<=72+2*15-1)) ||
										((ycnt==18)&&(zcnt>=71+2*7-1)&&(zcnt<=72+2*15-1)) ||
										((ycnt==17)&&(zcnt>=70+2*7-1)&&(zcnt<=71+2*15-1)) ||
										((ycnt==16)&&(zcnt>=71+2*7-1)&&(zcnt<=72+2*15-1)) ||
										((ycnt==15)&&(zcnt>=71+2*7-1)&&(zcnt<=72+2*15-1))  ) positionVector[ 3 ] += 0.00027f; 
									}//////////////////////////////////////////////////////////////////////


									if( ((int)(positionVector[ 3 ]*100000))%100 > 0)
									{
										if(xcnt>11) positionVector[ 3 ] += 0.00050f; // left / right
									}
								}

								if((xcnt==7)||(xcnt==15)) zcnt += -1;
								if((xcnt==10)||(xcnt==12)) zcnt += 1;
								}
								
								//if( (positionVector[ 3 ] <= 2.4+ 0.000005) && (positionVector[ 3 ] >= 2.4-0.000005) && (ycnt>=20) && (zcnt>10) ) //0.0001
								if((ycnt>=18)&&(xcnt!=11))
								{
									if( (((int)(positionVector[ 3 ]*100000))%100 <= 0) && (zcnt>9)) //not muscles, not head
									positionVector[ 3 ] -= 0.01;// 2.4 - 0.01 = 2.391f; // transparent area above muscles to see their activity
								} 

								if( ((int)(positionVector[ 3 ]*100000))%100 == 0)
								{
									if( (xx-11)*(xx-11)*0.62f + (yy-16)*(yy-16)*2      + (z-263)*(z-263)*0.4f  < 45/*52*/ ) positionVector[ 3 ] = 2.39;
								}

								if(within_body_shell==4) positionVector[ 3 ] = 2.35; //fins rigidity (stiffness) ribs

								//=====================================================
								//if((xx>10.95)&&(xx<11.05))
								//if(0==1)
								if(xcnt==11)
								{
									//if(0==1)
									{
									if( /*(xx-11)*(xx-11)*0.60f +*/ (ycnt-8.5+1)*(ycnt-8.5-5.5)*0.6f + (z-295-10)*(z-295-10)*0.020f < 52 )  positionVector[ 3 ] = 2.32f; //belly
									if( /*(xx-11)*(xx-11)*0.70f +*/ (ycnt-8.5+1)*(ycnt-8.5+1)*1.0f + (z-291-10)*(z-291-10)*0.045f < 52 )  positionVector[ 3 ] = 2.32f;
									if( /*(xx-11)*(xx-11)*0.55f +*/ (ycnt-8.0+1)*(ycnt-8.0+1)*0.95f + (z-288-10)*(z-288-10)*0.0450f < 52 )  positionVector[ 3 ] = 2.32f;
									}
									
									
									if((positionVector[ 3 ]>2.319f)&&(positionVector[ 3 ]<2.321f)) 
									{
										// belly
									///	printf("b");	
										positionVector[ 3 ] = 2.36; // vertical midplane belly area
									}
									else
									{
									///	printf("F");
										positionVector[ 3 ] = 2.34; //fins
									}
								}//=====================================================

								//=====================================================
								if(((xcnt==10)||(xcnt==11)||(xcnt==12))&&(ycnt==17)&&(zcnt>=5)&&(zcnt<=99))
								{
								///	printf("n");
									positionVector[ 3 ] = 2.33; //notochord
								}//=====================================================
								
								if(((xcnt==10)||(xcnt==11)||(xcnt==12))&&(ycnt==16)&&(zcnt>=5)&&(zcnt<=99))
								{
								///	printf("n");
									positionVector[ 3 ] = 2.33; //notochord
								}
								if(((xcnt==10)||(xcnt==11)||(xcnt==12))&&(ycnt==18)&&(zcnt>=5)&&(zcnt<=99))
								{
								///	printf("n");
									positionVector[ 3 ] = 2.33; //notochord
								}
								if(((xcnt==10)||(xcnt==11)||(xcnt==12))&&(ycnt==19)&&(zcnt>=5)&&(zcnt<=80-2))
								{
								///	printf("n");
									positionVector[ 3 ] = 2.33; //notochord
								}
								if((xcnt==11)&&(ycnt==20)&&(zcnt>=5)&&(zcnt<=60))
								{
								///	printf("n");
									positionVector[ 3 ] = 2.33; //notochord
								}
								

								/*
								positionVector[ 0 ] += 60;
								positionVector[ 1 ] -= 10;
								positionVector[ 2 ] -= 250;
								*/
								//positionVector[ 2 ] -= 200;

								//positionVector[ 0 ] += -10;
								positionVector[ 1 ] -= 18 + 18; // at the bottom
								positionVector[ 2 ] -= 1;
								//positionVector[ 2 ] -= 51.5;

								//flip the tadpole
								
								positionVector[ 2 ] = -positionVector[ 2 ] + 417+1;// + 20;// + 350;
								positionVector[ 1 ] -= /*-(23-8)-30;*/  23-8;//-10;

								/*
								tmp_x = positionVector[ 0 ];
								positionVector[ 0 ] = positionVector[1]+30 ;//-positionVector[ 1 ];
								positionVector[ 1 ] = -tmp_x+73; //tmp_x-42;
								*/


								velocityVector = velocity_cpp + 4 * (pCount+i_start);
								velocityVector[ 3 ] = zcnt;

							}
						}

						if(stage==0)
						{
							config->tadpole_y_min = std::min(y, config->tadpole_y_min);
							config->tadpole_y_max = std::max(y, config->tadpole_y_max);
							config->tadpole_z_min = std::min(z, config->tadpole_z_min);
							config->tadpole_z_max = std::max(z, config->tadpole_z_max);
						}
						else
						if(stage==1) //simple tadpole coloring
						{
							c_bmp = ((float)img_w)/(config->tadpole_z_max - config->tadpole_z_min);
							z_bmp = (z - config->tadpole_z_min) * c_bmp; 
							y_bmp = (y - config->tadpole_y_min) * c_bmp;
							//printf("\n%f: %f\n",c_bmp, c_bmp * (config->tadpole_y_max - config->tadpole_y_min));
							//printf("%d, %d\n",img_w,img_h);
							//printf("(%f, %f) - (%f, %f)\n",config->tadpole_y_min, config->tadpole_z_min, config->tadpole_y_max, config->tadpole_z_max);
							
							int img_z, img_y;
							int pixel_cnt = 0;
							int tc_r = 0, tc_g = 0, tc_b = 0;

							for(img_z = (int)(std::max(z_bmp - c_bmp*0.5f,0.f)); img_z < (int)(std::min(z_bmp + c_bmp*0.5f,(float)(img_w-1))); img_z++)
							{
								for(img_y = (int)(std::max(y_bmp - c_bmp*0.5f+6,0.f)); img_y < (int)(std::min(y_bmp + c_bmp*0.5f+6,(float)(img_h-1))); img_y++)
								{
									tc_r += img_data[img_y*img_w*3 + img_z*3+0];
									tc_g += img_data[img_y*img_w*3 + img_z*3+1];
									tc_b += img_data[img_y*img_w*3 + img_z*3+2];
									pixel_cnt++;
								}
							}

							if(pixel_cnt>0)
							{
								config->tadpole_color_r[pCount] = (unsigned char) (tc_r / pixel_cnt);
								config->tadpole_color_g[pCount] = (unsigned char) (tc_g / pixel_cnt);
								config->tadpole_color_b[pCount] = (unsigned char) (tc_b / pixel_cnt);
							}
						}

						pCount++;

						/*
						//second tadpole
						if(stage==1)
						{
							particle_type = positionVector[ 3 ];
							positionVector = position_cpp + 4 * (pCount+i_start);
							positionVector[ 0 ] = x;
							positionVector[ 1 ] = y;
							positionVector[ 2 ] = z - 230;
							positionVector[ 3 ] = particle_type;
							velocityVector = velocity_cpp + 4 * (pCount+i_start);
							velocityVector[ 3 ] = zcnt;
						}

						pCount++;
						/**/
					}
				}
			}
		}

	if( !((stage==0)||(stage==1)) ) return 0;

	if(stage==1)
	{
		for(int i=0;i<pCount;i++)
		{
			velocityVector = velocity_cpp + 4 * (i+i_start);	
			velocityVector[ 0 ] = 0;
			velocityVector[ 1 ] = 0;
			velocityVector[ 2 ] = 0;
			velocityVector[ 3 ] = 0;
		}
	}

	return pCount;

}

int generateLiquid(int stage, int i_start,float *position_cpp, float *velocity_cpp, owConfigProperty * config)
{
	//add outer liquid 2020

	int pCount = 0;//particle counter
	int i;

	float *positionVector;
	float *velocityVector;
	int elasticLayers;//starting from 2, because 1 is for outer shell and doesn't contain liquid particles
	float xc = config->xmax*0.50f;//0.61
	float yc = config->ymax*0.60f;//0.6f;
	float zc = config->zmax*0.55f;//35f;//39f;//0.45f;//0.36f;//0.45f;//0.26f;
	float PI = 3.1415926536f;
	float x,y,z;

	pCount = 0;
	
	float j;

	//and here we add outer liquid for tadpole swimming
	/**/
	int ycnt = 0;
	int xcnt = 0;
	int zcnt = 0;
	int overlap;
	float dr2,dr02=r0*r0,min_dr2;
	min_dr2 = 1000000;
	float sqrt3 = sqrt(3.f);
	float sqrt2d3 = sqrt(2.f/3.f);
	float r0s = r0*0.48f;

	float tadpole_xmin = 1000, tadpole_xmax = 0;
	float tadpole_ymin = 1000, tadpole_ymax = 0;
	float tadpole_zmin = 1000, tadpole_zmax = 0;

	if(stage==1)
	{
		for(i=0;i<numOfElasticP;i++)
		{
			tadpole_xmin = std::min(tadpole_xmin, position_cpp[ 4 * i + 0 ] ); tadpole_xmax = std::max( tadpole_xmax, position_cpp[ 4 * i + 0 ] );
			tadpole_ymin = std::min(tadpole_ymin, position_cpp[ 4 * i + 1 ] ); tadpole_ymax = std::max( tadpole_ymax, position_cpp[ 4 * i + 1 ] );
			tadpole_zmin = std::min(tadpole_zmin, position_cpp[ 4 * i + 2 ] ); tadpole_zmax = std::max( tadpole_zmax, position_cpp[ 4 * i + 2 ] );
		}

		for(x= 1.3f*r0 + 0*35*r0, xcnt = 0; x<config->xmax - 0*35*r0 - 1.3f*r0; x+=r0s*2.f*sqrt2d3, xcnt++)	
		{
			for(y= 1.3f*r0 + 0*15*r0, ycnt = 0; y<config->ymax*(0.93f-0.03f-0.27f) -0*15*r0 -1.3f*r0; y+=r0s*sqrt3, ycnt++)//ok
			{
				for(z= 1.3f*r0 + 0*10*r0 +(r0s)*(ycnt%2==1)+(r0s*2.f/sqrt3)*(xcnt%2==1),zcnt=0;z<config->zmax*1.0f -0*r0 - 1.3f*r0; z+=2*r0s,zcnt++) //ok	
				{
					overlap = 0;

					if( (tadpole_xmin-r0<=x)&&(x<=tadpole_xmax+r0)&&
						(tadpole_ymin-r0<=y)&&(y<=tadpole_ymax+r0)&&
						(tadpole_zmin-r0<=z)&&(z<=tadpole_zmax+r0) )
					{
						for(i=0;i<numOfElasticP;i++) // if (x,y,z) is inside the 3D box which surrounds the tadpole, then check for overlaps
						{
							dr2 = (position_cpp[ 4 * i + 0 ] - x)*(position_cpp[ 4 * i + 0 ] - x)
								+ (position_cpp[ 4 * i + 1 ] - y)*(position_cpp[ 4 * i + 1 ] - y)
								+ (position_cpp[ 4 * i + 2 ] - z)*(position_cpp[ 4 * i + 2 ] - z);

							min_dr2 = std::min(dr2,min_dr2);

							if(dr2<dr02*1) 
							{
								overlap=1; 
								break;
							}
						}
					}
				
					if(!overlap)
					{
						if(stage==1)
						{	
							positionVector = position_cpp + 4 * (pCount+i_start);
							positionVector[ 0 ] = x;//+r0*(-10+rand()%200)/10000.f;
							positionVector[ 1 ] = y;//+r0*(-10+rand()%200)/10000.f;
							positionVector[ 2 ] = z;//+r0*(-10+rand()%200)/10000.f;
							positionVector[ 3 ] = 1.1f;// liquid
							//if(xcnt>10)
							//positionVector[ 3 ] = 1.2f;// liquid
						}

						pCount++;
					}
				}
			}
		}
	} 
	else
	{
		pCount = 1949112;
	}
/**/ // end of block 

	if(stage==1)
	{
		for(i=0;i<pCount;i++)
		{
			velocityVector = velocity_cpp + 4 * (i+i_start);	
			velocityVector[ 0 ] = 0;
			velocityVector[ 1 ] = 0;
			velocityVector[ 2 ] = 0;
			velocityVector[ 3 ] = 0;
		}

		/*
		for(i=0;i<pCount;i++)
		{
			int ep_cnt = 0;
			positionVector  = position_cpp + 4 * (i+i_start);

			if( ((int)positionVector[ 3 ])==2)
			{
				//printf("here is the problem!");
				ep_cnt++;
			}

			ep_cnt += 0;
		}*/
	}

	return pCount;
}


unsigned char* ReadBMP(char* filename, int* img_w, int *img_h)
{
    int i;
    FILE* f = fopen(filename, "rb");

    if(f == NULL)
        throw "Argument Exception";

    unsigned char info[54];
    fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header

    // extract image height and width from header
    int width = *(int*)&info[18];
    int height = *(int*)&info[22];

    cout << endl;
    cout << "  Name:   " << filename << endl;
    cout << "  Width:  " << width << endl;
    cout << "  Height: " << height << endl;

	*img_h = height;
	*img_w = width;

    int row_padded = (width*3 + 3) & (~3);
    unsigned char* data = new unsigned char[row_padded];
	unsigned char* data2D = new unsigned char[width*height*3];
    unsigned char tmp;

    for(int i = 0; i < height; i++)
    {
        fread(data, sizeof(unsigned char), row_padded, f);
        for(int j = 0; j < width*3; j += 3)
        {
            // Convert (B, G, R) to (R, G, B)
            tmp = data[j];
            data[j] = data[j+2];
            data[j+2] = tmp;

            //cout << "R: "<< (int)data[j] << " G: " << (int)data[j+1]<< " B: " << (int)data[j+2]<< endl;
			data2D[i*3*width+j+0] = data[j+0];
			data2D[i*3*width+j+1] = data[j+1];
			data2D[i*3*width+j+2] = data[j+2];
        }
    }

    fclose(f);
	delete [] data;
    return data2D;
}

void owHelper::generateConfiguration(int stage, float *position_cpp, float *velocity_cpp, float *& elasticConnectionsData_cpp, int *membraneData_cpp, int & numOfLiquidP, int & numOfElasticP, int & numOfBoundaryP, int & numOfElasticConnections, int & numOfMembranes, int *particleMembranesList_cpp, owConfigProperty * config)
{
	float p_type = LIQUID_PARTICLE;
	int i = 0;// particle counter
	int ix,iy,iz;
	int ecc = 0;//elastic connections counter

	int nx = (int)( ( config->xmax - config->xmin ) / (r0) ); //X
	int ny = (int)( ( config->ymax - config->ymin ) / (r0) ); //Y
	int nz = (int)( ( config->zmax - config->zmin ) / (r0) ); //Z

	if(stage==0)
	{
		printf("Building tadpole body model...");
	/*int numOfMembraneParticles*/ numOfElasticP = generateTadpoleBody(0,0,position_cpp,velocity_cpp, numOfMembranes, membraneData_cpp, config);
		printf("Ok\n");

		printf("Loading tadpole body texture...");
		img_data = ReadBMP("tadpole.bmp", &img_w, &img_h);

		config->tadpole_color_r = new unsigned char [numOfElasticP];
		config->tadpole_color_g = new unsigned char [numOfElasticP];
		config->tadpole_color_b = new unsigned char [numOfElasticP];
		printf("Ok\n");

		printf("Generate liquid...");
		numOfLiquidP = generateLiquid(0,0,position_cpp,velocity_cpp, config);
		printf("Ok\n");
		//numOfElasticP = numOfElasticP; //numOfMembraneParticles;
		numOfBoundaryP = 0;

		if(numOfElasticP<=0) 
			elasticConnectionsData_cpp = NULL; 
		else 
			elasticConnectionsData_cpp = new float[ 4 * numOfElasticP * MAX_NEIGHBOR_COUNT ];
	}

	//=============== create body (elastic) ==================================================
	if(stage==1)
	{
		i += generateTadpoleBody(1/*stage*/,i,position_cpp,velocity_cpp, numOfMembranes,membraneData_cpp, config);
		//initialize elastic connections data structure (with NO_PARTICLE_ID values)
		for(int ii = 0; ii < numOfElasticP * MAX_NEIGHBOR_COUNT; ii++)
		{
			ecc = 0;

			elasticConnectionsData_cpp[ 4 * ii + 0 ] = NO_PARTICLE_ID;
			elasticConnectionsData_cpp[ 4 * ii + 1 ] = 0;
			elasticConnectionsData_cpp[ 4 * ii + 2 ] = 0;
			elasticConnectionsData_cpp[ 4 * ii + 3 ] = 0; 
		}
	}


	//=============== create liquid) ==================================================
	if(stage==1)
	{
		i += generateLiquid(stage,i,position_cpp,velocity_cpp, config);
	}


	if(stage==0) 
	{
		numOfBoundaryP = 2 * ( nx*ny + (nx+ny)*(nz)); 
	}
	else
	if(stage==1)
	{
		//===================== create boundary particles ==========================================================
		p_type = BOUNDARY_PARTICLE;

		int new_cnt = 0;
		
		// 1 - top and bottom 
		for(ix=0;ix<nx;ix++)
		{
			for(iy=0;iy<ny;iy++)
			{
				{
						position_cpp[ 4 * i + 0 ] = ix*r0 + r0/2;//x
						position_cpp[ 4 * i + 1 ] = iy*r0 + r0/2;//y
						position_cpp[ 4 * i + 2 ] =  0*r0 + r0/2;//z
						position_cpp[ 4 * i + 3 ] = p_type;
						velocity_cpp[ 4 * i + 0 ] =  0;//norm x
						velocity_cpp[ 4 * i + 1 ] =  0;//norm y
						velocity_cpp[ 4 * i + 2 ] =  1;//norm z
						velocity_cpp[ 4 * i + 3 ] = p_type;
						i++;
						position_cpp[ 4 * i + 0 ] = ix*r0 + r0/2;//x
						position_cpp[ 4 * i + 1 ] = iy*r0 + r0/2;//y
						position_cpp[ 4 * i + 2 ] = (nz-1)*r0 + r0/2;//z 
						position_cpp[ 4 * i + 3 ] = p_type;
						velocity_cpp[ 4 * i + 0 ] =  0;//norm x
						velocity_cpp[ 4 * i + 1 ] =  0;//norm y
						velocity_cpp[ 4 * i + 2 ] = -1;//norm z
						velocity_cpp[ 4 * i + 3 ] = p_type;
						i++;

						/*
						if(iy<4)
						{
							position_cpp[ 4 * i + 0 ] = ix*r0 + r0/2;//x
							position_cpp[ 4 * i + 1 ] = iy*r0 + r0/2;//y
							position_cpp[ 4 * i + 2 ] =  (nz-5)/2*r0 + r0/2;//z
							position_cpp[ 4 * i + 3 ] = p_type;
							velocity_cpp[ 4 * i + 0 ] =  0;//norm x
							velocity_cpp[ 4 * i + 1 ] =  0;//norm y
							velocity_cpp[ 4 * i + 2 ] =  1;//norm z
							velocity_cpp[ 4 * i + 3 ] = p_type;
							i++;
							new_cnt++;
						}/**/
				}
			}
		}

		// 2 - side walls OX-OZ and opposite
		for(ix=0;ix<nx;ix++)
		{
			for(iz=0/*1*/;iz<nz-0/*1*/;iz++)
			{
				{
					position_cpp[ 4 * i + 0 ] = ix*r0 + r0/2;//x
					position_cpp[ 4 * i + 1 ] =  0*r0 + r0/2;//y
					position_cpp[ 4 * i + 2 ] = iz*r0 + r0/2;//z
					position_cpp[ 4 * i + 3 ] = p_type;
					velocity_cpp[ 4 * i + 0 ] =  0;//norm x
					velocity_cpp[ 4 * i + 1 ] =  1;//norm y
					velocity_cpp[ 4 * i + 2 ] =  0;//norm z
					velocity_cpp[ 4 * i + 3 ] = p_type;
					i++;
					position_cpp[ 4 * i + 0 ] = ix*r0 + r0/2;//x
					position_cpp[ 4 * i + 1 ] = (ny-1)*r0 + r0/2;//y
					position_cpp[ 4 * i + 2 ] = iz*r0 + r0/2;//z
					position_cpp[ 4 * i + 3 ] = p_type;
					velocity_cpp[ 4 * i + 0 ] =  0;//norm x
					velocity_cpp[ 4 * i + 1 ] = -1;//norm y
					velocity_cpp[ 4 * i + 2 ] =  0;//norm z
					velocity_cpp[ 4 * i + 3 ] = p_type;
					i++;

					//     |Y 
					//     |
					//     |_______ Z
					//    /
					//  X/

					/*
					if(iz<-2+nz/2)
					{
						position_cpp[ 4 * i + 0 ] = ix*r0 + r0/2;//x
						position_cpp[ 4 * i + 1 ] =  3.5*r0 + r0/2;//y
						position_cpp[ 4 * i + 2 ] = iz*r0 + r0/2;//z
						position_cpp[ 4 * i + 3 ] = p_type;
						velocity_cpp[ 4 * i + 0 ] =  0;//norm x
						velocity_cpp[ 4 * i + 1 ] =  1;//norm y
						velocity_cpp[ 4 * i + 2 ] =  0;//norm z
						velocity_cpp[ 4 * i + 3 ] = p_type;
						i++;
						new_cnt++;
					}/**/
				}
			}
		}

		// 3 - side walls OY-OZ and opposite
		for(iy=0/*1*/;iy<ny-0/*1*/;iy++)
		{
			for(iz=0/*1*/;iz<nz-0/*1*/;iz++)
			{
				position_cpp[ 4 * i + 0 ] =  0*r0 + r0/2;//x
				position_cpp[ 4 * i + 1 ] = iy*r0 + r0/2;//y
				position_cpp[ 4 * i + 2 ] = iz*r0 + r0/2;//z
				position_cpp[ 4 * i + 3 ] = p_type;
				velocity_cpp[ 4 * i + 0 ] =  1;//norm x
				velocity_cpp[ 4 * i + 1 ] =  0;//norm y
				velocity_cpp[ 4 * i + 2 ] =  0;//norm z
				velocity_cpp[ 4 * i + 3 ] = p_type;
				i++;
				//position_cpp[ 4 * i + 0 ] = (nx*4.7/8-1)*r0 + r0/2;//x
				position_cpp[ 4 * i + 0 ] = (nx-1)*r0 + r0/2;//x
				position_cpp[ 4 * i + 1 ] = iy*r0 + r0/2;//y
				position_cpp[ 4 * i + 2 ] = iz*r0 + r0/2;//z
				position_cpp[ 4 * i + 3 ] = p_type;
				velocity_cpp[ 4 * i + 0 ] = -1;//norm x
				velocity_cpp[ 4 * i + 1 ] =  0;//norm y
				velocity_cpp[ 4 * i + 2 ] =  0;//norm z
				velocity_cpp[ 4 * i + 3 ] = p_type;
				i++;
			}
		}

		//std::ifstream tadpole_surface_file ("half_tadpole_points2.xyz", std::ios_base::binary);
/*		std::ifstream tadpole_surface_file ("3d_form_for_tadpole.xyz", std::ios_base::binary);
		float x,y,z,point_type;
		int n_points;
		int n_midplane_point = 0;

		if( tadpole_surface_file.is_open() )
		{
			tadpole_surface_file >> n_points;
			n_points = 0;

			while( tadpole_surface_file.good() )
			{
				tadpole_surface_file >> x;
				tadpole_surface_file >> y;
				tadpole_surface_file >> z;
				tadpole_surface_file >> point_type;

				position_cpp[ 4 * i + 0 ] = x; //config->xmax*0.50f + x*34*0.682;
				position_cpp[ 4 * i + 1 ] = y; //config->ymax*0.46f + y*34*0.682;
				position_cpp[ 4 * i + 2 ] = z; //config->zmax*0.17f + z*34*0.682;// 0.5-->0.2 -- shift along anterior-posterior axis
				position_cpp[ 4 * i + 3 ] = point_type + 1;//p_type+0.15f;
				velocity_cpp[ 4 * i + 0 ] =  0;//norm x
				velocity_cpp[ 4 * i + 1 ] =  0;//norm y
				velocity_cpp[ 4 * i + 2 ] =  0;//norm z
				velocity_cpp[ 4 * i + 3 ] = point_type + 1; //p_type+0.15f;
				i++;*/

				/*
				if(x==0) 
					n_midplane_point++;			
				else
				{
					position_cpp[ 4 * i + 0 ] = config->xmax*0.50f - x*34*0.682;
					position_cpp[ 4 * i + 1 ] = config->ymax*0.46f + y*34*0.682;
					position_cpp[ 4 * i + 2 ] = config->zmax*0.17f + z*34*0.682;// 0.5-->0.2 -- shift along anterior-posterior axis
					position_cpp[ 4 * i + 3 ] = p_type+0.15f;
					velocity_cpp[ 4 * i + 0 ] =  0;//norm x
					velocity_cpp[ 4 * i + 1 ] =  0;//norm y
					velocity_cpp[ 4 * i + 2 ] =  0;//norm z
					velocity_cpp[ 4 * i + 3 ] = p_type+0.15f;
					i++;
				}/**/
		/*
			}

			tadpole_surface_file.close();
		}*/
	}

	if(stage==0)
	{
		config->setParticleCount(numOfLiquidP + numOfBoundaryP + numOfElasticP);

		if(config->getParticleCount()<=0)
		{
			printf("\nWarning! Generated scene contains %d particles!\n",config->getParticleCount());
			exit(-2);
		}
	}
	else
	if(stage==1)
	{
		if(config->getParticleCount()!=i)
		{
			printf("\nWarning! Preliminary [%d] and final [%d] particle count are different\n",config->getParticleCount(),i);
			exit(-4);
		}

		
		for(int mli = 0/*membrane list index*/; mli<numOfElasticP; mli++)
		{
			for(int sli = 0 /*sublist index*/; sli<MAX_MEMBRANES_INCLUDING_SAME_PARTICLE; sli++)
			{
				particleMembranesList_cpp [mli*MAX_MEMBRANES_INCLUDING_SAME_PARTICLE + sli] = -1;// no membranes connected with current particle
			}
		}

	///////////////debug////////////
	int j;
	//int ecc_total = 0;
	//float ix,iy,iz,jx,jy,jz;
	//int j_count=0;
	int muscleCounter = 0;
	//int m_index[10640];
	float WXC = config->xmax*0.50f;//0.61
	float WYC = config->ymax*0.60f;//0.47f;//0.6f;
	float WZC = config->zmax*0.55f;//35f;//39f;//0.45f;//0.36f;//0.45f;//0.26f;
	float dx2,dy2,dz2,r2_ij,r_ij;
	int ecc_j;

	int n_ijmc = 600;
	int i_j_m_c [600][4];
	
	int ij_mp_cnt = 0;

		//float xmax = -100,xmin = 100;

		//int array_k[MAX_NEIGHBOR_COUNT];
		//for(i=numOfElasticP-numOfMembraneParticles;i<numOfElasticP;i++)
		for(i=0;i<numOfElasticP+numOfLiquidP*0+numOfBoundaryP*0;i++)
		{
			if(i==numOfElasticP) i+= numOfLiquidP;

			//xmax = max(xmax, position_cpp[ 4 * i + 2 ]);
			//xmin = min(xmin, position_cpp[ 4 * i + 2 ]);
			
			int q_i_start;
			int dq;//dorsal quadrant - "+1"=right, "-1"=left
			float muscle_color = 0.1f;
			float r0_ = r0*0.5f;;
			ecc = 0;//!important!
			//        _____1_______      2       _____3________       
			for(j=0;j<numOfElasticP+numOfLiquidP*0+numOfBoundaryP*1;j++)
			{
				if((i!=j) )
				{
					if(j==numOfElasticP) j+= numOfLiquidP;//skip liquid particles (they are located in the middle of memory) as candidates to spring connections


					dx2 = (position_cpp[ 4 * i + 0 ] - position_cpp[ 4 * j + 0 ]); dx2 *= dx2;
					dy2 = (position_cpp[ 4 * i + 1 ] - position_cpp[ 4 * j + 1 ]); dy2 *= dy2;
					dz2 = (position_cpp[ 4 * i + 2 ] - position_cpp[ 4 * j + 2 ]); dz2 *= dz2;
					r2_ij = dx2 + dy2 + dz2;
					r_ij = (float)sqrt(r2_ij);


					//if(r_ij<=r0*0.1)
					//if(r_ij<=r0*sqrt(1.5))
					int add_muscle = 0;
					int mj,mi = 0;
					int mi_pos[8] = {10,56,104,154,205,248,282,309};
					float mi_color;
					int mus_segm_cnt = 0;
					int found;
					int ij_belong_to_midplane = 0;
					int m_index_i, m_index_j;
					

					if( (fabs(position_cpp[ 4 * i + 0 ] - 73.f + 16.f) < 0.01f) && (fabs(position_cpp[ 4 * j + 0 ] - 73.f + 16.f) < 0.01f) )  
					{
						ij_belong_to_midplane = 1;
						ij_mp_cnt++;
					}

					
					//if( (r_ij>=r0*0.8) && (r_ij<=r0*1.9) )

					//if(r_ij<=r0*sqrt(2.4+0.6*(ij_belong_to_midplane)))//2.8//2.7
					if(r_ij<=r0*sqrt(2.6))
					//if(r_ij<=r0*sqrt(3.5))
					//if(muscleCounter < 1100-12 )
					//if(0==1)
					{
						ecc = elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i ) + 3 ];
						elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + ecc) + 0 ] = ((float)j) + 0.05f;		// index of j-th particle in a pair connected with spring
						elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + ecc) + 1 ] = r_ij*simulationScale*1.0f;	// resting distance; that's why we use float type for elasticConnectionsData_cpp
						elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + ecc) + 2 ] = 0;						// type of connection; 0 - ordinary spring, 1 - muscle
						elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + ecc) + 3 ] = 0;						// not in use yet

						m_index_i = ((int)((position_cpp[ 4 * i + 3 ]/*+0.000001*/) * 100000))%100;
						m_index_j = ((int)((position_cpp[ 4 * j + 3 ]/*+0.000001*/) * 100000))%100;

						//getIteration();

						//muscle mapping 2.2
						if( (abs(m_index_i-m_index_j)<=1) && (m_index_i>0) && (m_index_j>0) 
							&& (position_cpp[4*i+0]==position_cpp[4*j+0]) // x
							&& (position_cpp[4*i+1]==position_cpp[4*j+1]))//y
						{
							m_index_i = std::max(m_index_i,m_index_j);
							elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + ecc  ) + 2 ] = m_index_i;// + 0*((m_index_i)%2+2)/10.f; //mus_clr[mus_num]/10.f;
						}

						

						//if( ((int)(position_cpp[ 4 * i + 3 ]*10)==25) && ((int)(position_cpp[ 4 * j + 3 ]*10)==25) ) // type == 2.5 for muscles, 2.4 - for ordinary elastic matter
							/*
						{
								found = 0;

								for(int k=0; k<n_ijmc; k++)
								{
									if( (i==i_j_m_c[k][0]+0) && (j==i_j_m_c[k][1]+0) ) 
									{
											elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + ecc  ) + 2 ] = i_j_m_c[k][2] + i_j_m_c[k][3]/10.f;
									}
									if( (j==i_j_m_c[k][0]+0) && (i==i_j_m_c[k][1]+0) ) 
									{
										elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + ecc  ) + 2 ] = i_j_m_c[k][2] + i_j_m_c[k][3]/10.f;
									}
								}
						}*/

						elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i ) + 3 ]++;
						//ecc++;
						//ecc_total++;
						
					}
				}
			}

			//printf("\nij_mp_cnt = %d\n", ij_mp_cnt);
		}

		//int muscle_map [50] = {};
		int mc;
		int q4 = 0;
//		int mus_num;
//		int n_mus_i, n_mus_j;

		/*
		for(i = 6049, mc = 2, mus_num = 1, q4 = 0; i< 6049+4*(44); i++, q4++)
		{
			if(q4>=8) { mc++; q4 = 0; mus_num++; }
			if(mc>5) mc = 2;
			j = i+4;

			n_mus_i = (int) elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + 0) + 3 ];
			n_mus_j = (int) elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * j + 0) + 3 ];

			elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + n_mus_i) + 0 ] = j + 0.05f;
			elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + n_mus_i) + 2 ] = mus_num*2 - 1*((q4==0)||(q4==3))*1 + 0.1f*mc;// type of connection; 0 - ordinary spring, 1 or more - muscle

			elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * j + n_mus_j) + 0 ] = i + 0.05f;
			elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * j + n_mus_j) + 2 ] = mus_num*2 - 1*((q4==0)||(q4==3))*1  + 0.1f*mc;

			elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + 0) + 3 ] = n_mus_i + 1.05f;
			elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * j + 0) + 3 ] = n_mus_j + 1.05f;
		}
		/**/

		//ecc = 0;
		/*
		for(i = 6237, mc = 2, mus_num = 1, q4 = 0; i< 6237+4*44; i++, q4++)
		{
			if(q4>=8) { mc++; q4 = 0; mus_num++; }
			if(mc>5) mc = 2;
			j = i+4;

			elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + ecc) + 0 ] = j + 0.05f;
			elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + ecc) + 2 ] = mus_num*2-1*((q4==0)||(q4==3)) + 0.1f*mc;// type of connection; 0 - ordinary spring, 1 or more - muscle

			elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * j + ecc) + 0 ] = i + 0.05f;
			elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * j + ecc) + 2 ] = mus_num*2-1*((q4==0)||(q4==3))  + 0.1f*mc;
		}
		/**/

		/*
		i = 6050;
		j = 6054;

		elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + ecc) + 0 ] = j + 0.05f;
		elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * i + ecc) + 2 ] = 0 + 1.2;// type of connection; 0 - ordinary spring, 1 or more - muscle

		elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * j + ecc) + 0 ] = i + 0.05f;
		elasticConnectionsData_cpp[ 4 * ( MAX_NEIGHBOR_COUNT * j + ecc) + 2 ] = 0 + 1.2;
		*/
		
		/*
		if(muscleCounter==10408)
		{
			//for(i=0;i<muscleCounter;i++)

			//m_number[m_index[0]] = 1.2f;
			elasticConnectionsData_cpp[m_index[3]] = 1.2f;

		}*/


		//membraneData - the list containing triplets of indexes of particles forming triangular membranes; size: numOfMembranes*3
		//particleMembranesList - the list containing for each particle(involved in membranes) its list of these membranes indexes (which are stored in membraneData array)

		for(int _mc = 0; _mc < numOfMembranes*3; _mc++)
		{
			int particle_index;
			for(int sli=0/*sublist index*/;sli<MAX_MEMBRANES_INCLUDING_SAME_PARTICLE; sli++)
			{//search for the not filled yet cell (-1) in the list and fill it; 
				if(/**/particleMembranesList_cpp [particle_index = membraneData_cpp [_mc]*MAX_MEMBRANES_INCLUDING_SAME_PARTICLE+sli]/**/==-1)
				{
					particleMembranesList_cpp [particle_index] = _mc/3;
					break;//if there are no free cells, break, because we are limited with MAX_MEMBRANES_INCLUDING_SAME_PARTICLE per particle
				} 
				else
				{

				}
			}		
		}
	}



	return;
}


//READ DEFAULT CONFIGURATATION FROM FILE IN CONFIGURATION FOLDER
int read_position = 0;
std::string owHelper::path = "./configuration/";
std::string owHelper::configFileName = "demo1";

/** TODO make description
 *
 */
void findValf(std::string & str, char delimiter, size_t & start, float & val){
	size_t end = str.find(delimiter, start + 1);
	if(end != std::string::npos){
		val =std::atof(str.substr(start, end - start).c_str());//TODO make check if this a number
		start = end;
	}else // it means usualy that we reached the end of string and there are no \t
		val = std::atof(str.substr(start).c_str());//TODO make check if this a number
}
/** TODO Documentation
 */
void findVali(std::string & str, char delimiter, size_t & start, int & val){
	size_t end = str.find(delimiter, start + 1);
	if(end != std::string::npos){
		val =std::atoi(str.substr(start, end - start).c_str());
		start = end;
	}else // it means usualy that we reached the end of string and there are no \t
		val = std::atoi(str.substr(start).c_str());
}
/** Preparing initial data before load full configuration
 *
 *  Before load configuration data from file (initial position and velocity,
 *  connection data if is and membrane data if is) Sibernetic
 *  should allocate a memory in RAM. Method starts with reading position file
 *  first 6 lines in file correspond to dimensions of boundary box, than it reads all file
 *  till the end an calculate numbers of elastic, liquid and boundary
 *  particles and total number too. Also this read membranes file
 *  for counting membranes numbers.
 *
 *  @param numOfMembranes
 *  reference to numOfMembrane variable
 *  @param config
 *  pointer to owConfigProperty object it includes information about
 *  boundary box dimensions
 *  @param numOfLiquidP
 *  reference to numOfLiquidP variable
 *  @param numOfElasticP
 *  reference to numOfElasticP variable
 *  @param numOfBoundaryP
 *  reference to numOfBoundaryP variable
 */
void owHelper::preLoadConfiguration(int & numOfMembranes, owConfigProperty * config, int & numOfLiquidP, int & numOfElasticP, int & numOfBoundaryP)
{
	try
	{
		int p_count = 0;
		std::string file_name = path + config->getCofigFileName();
		std::string inputStr;
		std::ifstream configFile (file_name.c_str(), std::ios_base::binary);
		float x, y, z, p_type;
		char delimiter = '\t';
		size_t pos;
		ELOADMODE mode = NOMODE;
		if( configFile.is_open() )
		{
			configFile >> config->xmin;
			configFile >> config->xmax;
			configFile >> config->ymin;
			configFile >> config->ymax;
			configFile >> config->zmin;
			configFile >> config->zmax;
			read_position = configFile.tellg();
			while( configFile.good() )
			{
				//configFile >> inputStr;
				std::getline(configFile,inputStr);
				if(inputStr == "[position]"){
					mode = POSITION;
					continue;
				}
				if(inputStr == "[velocity]"){
					mode = VELOCITY;
					continue;
				}
				if(inputStr == "[membranes]"){
					mode = MEMBRANE;
					continue;
				}
				if(inputStr == "[particleMemIndex]"){
					break;
				}
				switch(mode){
					case POSITION:{
						p_type = -1.1f;//reinitialize
						pos = 0;
						findValf(inputStr, delimiter,pos,x);
						findValf(inputStr, delimiter,pos,y);
						findValf(inputStr, delimiter,pos,z);
						findValf(inputStr, delimiter,pos,p_type);
						p_count++;
						switch((int)p_type){
							case LIQUID_PARTICLE:
								numOfLiquidP++;
								break;
							case ELASTIC_PARTICLE:
								numOfElasticP++;
								break;
							case BOUNDARY_PARTICLE:
								numOfBoundaryP++;
								break;
						}
						break;
					}
					case MEMBRANE:{
						numOfMembranes++;
						break;
					}
					default:
						continue;
				}
			}
		}else
			throw std::runtime_error("Could not open file configuration file");
		configFile.close();
		config->setParticleCount(p_count);
	}
	catch(std::exception &e){
		std::cout << "ERROR_02: " << e.what() << std::endl;
		exit( -2 );
	}
}
/** Load full configuration
 *
 *  Load configuration data from file (initial position and velocity,
 *  connection data if is and membrane data if is). Method starts with
 *  reading position file than velocity elastic connection and membranes data files
 *
 *  @param position_cpp
 *  pointer to position_cpp buffer
 *  @param velocity_cpp
 *  pointer to velocity_cpp buffer
 *  @param elasticConnections
 *  reference on pointer to elasticConnections buffer.
 *  In this function we allocate memory for elasticConnections.
 *  TODO: change it replace to owPhysicsFluidSimulator constructor.
 *  @param numOfLiquidP
 *  reference to numOfLiquidP variable
 *  @param numOfElasticP
 *  reference to numOfElasticP variable
 *  @param numOfBoundaryP
 *  reference to numOfBoundaryP variable
 *  @param numOfElasticConnections
 *  reference to numOfElasticConnections variable
 *  @param numOfMembranes
 *  reference to numOfMembranes variable
 *  @param membraneData_cpp
 *  pointer to membraneData_cpp buffer
 *  @param particleMembranesList_cpp
 *  pointer to particleMembranesList_cpp buffer
 *  @param config
 *  pointer to owConfigProperty object it includes information about
 */
void owHelper::loadConfiguration(float *position_cpp, float *velocity_cpp, float *& elasticConnections,int & numOfLiquidP, int & numOfElasticP, int & numOfBoundaryP, int & numOfElasticConnections, int & numOfMembranes,int * membraneData_cpp, int *& particleMembranesList_cpp, owConfigProperty * config)
{
	try
	{
		std::string file_name = path + config->getCofigFileName();
		std::string inputStr;
		std::ifstream configFile (file_name.c_str(), std::ios_base::binary);
		char delimiter = '\t';
		size_t pos;
		ELOADMODE mode = NOMODE;
		int i = 0;
		if( configFile.is_open() )
		{
			configFile.seekg(read_position);
			while( configFile.good() )
			{
				//configFile >> inputStr;
				std::getline(configFile,inputStr);
				if(inputStr == "[position]"){
					mode = POSITION;
					continue;
				}
				if(inputStr == "[velocity]"){
					i = 0;
					mode = VELOCITY;
					continue;
				}
				if(inputStr == "[connection]"){
					i = 0;
					mode = CONNECTION;
					continue;
				}
				if(inputStr == "[membranes]"){
					i = 0;
					mode = MEMBRANE;
					continue;
				}
				if(inputStr == "[particleMemIndex]"){
					i = 0;
					mode = PMEMINDEX;
					continue;
				}
				if(inputStr == "[end]")
					break;
				switch(mode){
					case POSITION:{
						float x, y, z, p_type;
						p_type = -1.1f;//reinitialize
						pos = 0;
						findValf(inputStr, delimiter,pos,x);
						findValf(inputStr, delimiter,pos,y);
						findValf(inputStr, delimiter,pos,z);
						findValf(inputStr, delimiter,pos,p_type);
						position_cpp[ 4 * i + 0 ] = x;
						position_cpp[ 4 * i + 1 ] = y;
						position_cpp[ 4 * i + 2 ] = z;
						position_cpp[ 4 * i + 3 ] = p_type;
						i++;
						break;
					}
					case VELOCITY:{
						float x, y, z, p_type;
						p_type = -1.1f;//reinitialize
						pos = 0;
						findValf(inputStr, delimiter,pos,x);
						findValf(inputStr, delimiter,pos,y);
						findValf(inputStr, delimiter,pos,z);
						findValf(inputStr, delimiter,pos,p_type);
						velocity_cpp[ 4 * i + 0 ] = x;
						velocity_cpp[ 4 * i + 1 ] = y;
						velocity_cpp[ 4 * i + 2 ] = z;
						velocity_cpp[ 4 * i + 3 ] = p_type;
						i++;
						break;
					}
					case CONNECTION:{
						pos = 0;
						float  jd, rij0, val1, val2;
						findValf(inputStr, delimiter,pos,jd);
						findValf(inputStr, delimiter,pos,rij0);
						findValf(inputStr, delimiter,pos,val1);
						findValf(inputStr, delimiter,pos,val2);
						elasticConnections[ 4 * i + 0 ] = jd;
						elasticConnections[ 4 * i + 1 ] = rij0 * simulationScale;
						elasticConnections[ 4 * i + 2 ] = val1;
						elasticConnections[ 4 * i + 3 ] = val2;
						i++;
						break;
					}
					case MEMBRANE:{
						pos = 0;
						int id, jd, kd;
						findVali(inputStr, delimiter,pos,id);
						findVali(inputStr, delimiter,pos,jd);
						findVali(inputStr, delimiter,pos,kd);
						membraneData_cpp[ 3 * i + 0 ] = id;
						membraneData_cpp[ 3 * i + 1 ] = jd;
						membraneData_cpp[ 3 * i + 2 ] = kd;
						i++;
						break;
					}
					case PMEMINDEX:{
						int id;
						pos = 0;
						findVali(inputStr, delimiter,pos,id);
						particleMembranesList_cpp[ i ] = id;
						i++;
						break;
					}
					default:
						continue;
				}
			}
		}else
			throw std::runtime_error("Could not open file configuration file");
		configFile.close();
		std::cout << "Configuration was loaded" << std::endl;
	}catch(std::exception &e){
		std::cout << "ERROR_03: " << e.what() << std::endl;
		exit( -3 );
	}
}
template<typename T>
std::ostream& binary_write(std::ostream& stream, const T& value){
    return stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
}
/** Load configuration from simulation to files
 *
 *  This method is required for work with "load config to file" mode.
 *  In this mode information about simulation's evolution is storing into a file
 *  on every step (every time it reads data block with size = c).
 *  If Sibernetic runs in this mode it means that
 *  no calculation on OpenCL device runs.
 *
 *  @param position
 *  pointer to position buffer
 *  @param config
 *  pointer to owConfigProperty object it includes information about
 *  @param connections
 *  reference on pointer to elasticConnections buffer.
 *  @param membranes
 *  pointer to membranes buffer
 *  @param firstIteration
 *  if true it means that we first time record information
 *  to a file and on first iteration it put to
 *  the file info about dimensions of boundary box
 *  NOTE: next 2 parameters are an experimental
 *  @param filter_p
 *  pointer to filter particle buffer, if you need storing only
 *  a bunch of particles not all of them
 *  @param size
 *  size of filter_p array
 */
void owHelper::loadConfigurationToFile(float * position, owConfigProperty * config, std::vector<int> & filter, float * connections, int * membranes, bool firstIteration){
	try{
		ofstream positionFile;
		if(firstIteration){
#if !EXPEREMENTAL_WRITE
			positionFile.open("./buffers/position_buffer.txt", ios::trunc);
			positionFile << config->xmin << "\n";
			positionFile << config->xmax << "\n";
			positionFile << config->ymin << "\n";
			positionFile << config->ymax << "\n";
			positionFile << config->zmin << "\n";
			positionFile << config->zmax << "\n";
#else
			positionFile.open("./buffers/position_buffer.txt", ios::trunc|ios::binary);
			binary_write(positionFile,config->xmin);
			binary_write(positionFile,config->xmax);
			binary_write(positionFile,config->ymin);
			binary_write(positionFile,config->ymax);
			binary_write(positionFile,config->zmin);
			binary_write(positionFile,config->zmax);
			binary_write(positionFile,40000.0f);
#endif
			if(!filter.empty()){
#if !EXPEREMENTAL_WRITE
				positionFile << filter.size() << "\n";
				positionFile << 0 << "\n";
#else
				binary_write(positionFile,(float)filter.size());
				binary_write(positionFile,0.0f);
#endif
			}
			else{
#if !EXPEREMENTAL_WRITE
				positionFile << numOfElasticP << "\n";
				positionFile << numOfLiquidP << "\n";
#else
				binary_write(positionFile,numOfElasticP);
				binary_write(positionFile,numOfLiquidP);
#endif
			}
		}else{
#if !EXPEREMENTAL_WRITE
			positionFile.open("./buffers/position_buffer.txt", ios::app);
#else
			positionFile.open("./buffers/position_buffer.txt", ios::app|ios::binary);
#endif
		}
		if(filter.empty()){
			for(int i=0;i < config->getParticleCount(); i++){
				if((int)position[ 4 * i + 3] != BOUNDARY_PARTICLE){
#if !EXPEREMENTAL_WRITE
				positionFile << position[i * 4 + 0] << "\t" << position[i * 4 + 1] << "\t" << position[i * 4 + 2] << "\t" << position[i * 4 + 3] << "\n";
#else
				binary_write(positionFile,position[i * 4 + 0]);
				binary_write(positionFile,position[i * 4 + 1]);
				binary_write(positionFile,position[i * 4 + 2]);
				binary_write(positionFile,position[i * 4 + 3]);
#endif

				}
			}
		}else{
			int i = 0;
			for(unsigned int index = 0; index<filter.size(); index++){
				i = filter[index];
#if !EXPEREMENTAL_WRITE
				positionFile << position[i * 4 + 0] << "\t" << position[i * 4 + 1] << "\t" << position[i * 4 + 2] << "\t" << position[i * 4 + 3] << "\n";
#else
				binary_write(positionFile,position[i * 4 + 0]);
				binary_write(positionFile,position[i * 4 + 1]);
				binary_write(positionFile,position[i * 4 + 2]);
				binary_write(positionFile,position[i * 4 + 3]);
#endif
			}
		}
		positionFile.close();
		if(firstIteration){
			ofstream connectionFile("./buffers/connection_buffer.txt", std::ofstream::trunc);
			int con_num = MAX_NEIGHBOR_COUNT * numOfElasticP;
			for(int i = 0; i < con_num; i++)
				connectionFile << connections[4 * i + 0] << "\t" << connections[4 * i + 1] << "\t" << connections[4 * i + 2] << "\t" << connections[4 * i + 3] << "\n";
			connectionFile.close();
			ofstream membranesFile("./buffers/membranes_buffer.txt", std::ofstream::trunc);
			membranesFile << numOfMembranes << "\n";
			for(int i = 0; i < numOfMembranes; i++)
				membranesFile << membranes[3 * i + 0] << "\t" << membranes[3 * i + 1] << "\t" << membranes[3 * i + 2] << "\n";
			membranesFile.close();
		}
	}catch(std::exception &e){
		std::cout << "ERROR_04: " << e.what() << std::endl;
		exit( -4 );
	}
}
/** Load configuration from simulation to files
 *
 *  TODO make description
 *
 *
 *
 *
 *
 *  @param position
 *  pointer to position buffer
 *  @param config
 *  pointer to owConfigProperty object it includes information about
 *  @param connections
 *  reference on pointer to elasticConnections buffer.
 *  @param membranes
 *  pointer to membranes buffer
 *  @param firstIteration
 *  if true it means that we first time record information
 *  to a file and on first iteration it put to
 *  the file info about dimensions of boundary box
 *  NOTE: next 2 parameters are an experimental
 *  @param filter_p
 *  pointer to filter particle buffer, if you need storing only
 *  a bunch of particles not all of them
 *  @param size
 *  size of filter_p array
 */
void owHelper::loadConfigurationToFile(float * position, float * velocity, float * connections, int * membranes, int * particleMemIndex,const char * filename, owConfigProperty * config){
	try{
		ofstream configFile;
		configFile.open(filename, std::ofstream::trunc);
		configFile << config->xmin << "\n";
		configFile << config->xmax << "\n";
		configFile << config->ymin << "\n";
		configFile << config->ymax << "\n";
		configFile << config->zmin << "\n";
		configFile << config->zmax << "\n";
		configFile << "[position]\n" ;
		for(int i=0;i < config->getParticleCount(); i++)
			configFile << position[i * 4 + 0] << "\t" << position[i * 4 + 1] << "\t" << position[i * 4 + 2] << "\t" << position[i * 4 + 3] << "\n";
		configFile << "[velocity]\n" ;
		for(int i=0;i < config->getParticleCount(); i++)
			configFile << velocity[i * 4 + 0] << "\t" << velocity[i * 4 + 1] << "\t" << velocity[i * 4 + 2] << "\t" << velocity[i * 4 + 3] << "\n";
		configFile << "[connection]\n" ;
		int con_num = MAX_NEIGHBOR_COUNT * numOfElasticP;
		for(int i = 0; i < con_num; i++)
			configFile << connections[4 * i + 0] << "\t" << connections[4 * i + 1] / simulationScale << "\t" << connections[4 * i + 2] << "\t" << connections[4 * i + 3] << "\n";
		configFile << "[membranes]\n";
		for(int i = 0; i < numOfMembranes; i++)
			configFile << membranes[3 * i + 0] << "\t" << membranes[3 * i + 1] << "\t" << membranes[3 * i + 2] << "\n";
		configFile << "[particleMemIndex]\n";
		int particleMemIndexCount = numOfElasticP*MAX_MEMBRANES_INCLUDING_SAME_PARTICLE;
		for(int i = 0; i < particleMemIndexCount; i++)
			configFile << particleMemIndex[i] << "\n";
		configFile << "[end]";
		configFile.close();
	}catch(std::exception &e){
		std::cout << "ERROR_05: " << e.what() << std::endl;
		exit( -5 );
	}
}
//This function needed for visualiazation buffered data
long position_index = 0;
ifstream positionFile;
/** Load configuration from file to simulation
 *
 *  This method is required for work with "load config from file" mode.
 *  In this mode information about simulation's evolution is taking from file
 *  on every step (every time it reads data block with size = PARTICLE_COUNT).
 *  If Sibernetic runs in this mode it means that
 *  no calculation on OpenCL device runs.
 *
 *  @param position
 *  pointer to position buffer
 *  @param connections
 *  reference on pointer to elasticConnections buffer.
 *  @param membranes
 *  pointer to membranes buffer
 *  @param config
 *  pointer to owConfigProperty object it includes information about
 *  @param iteration
 *  if iteration==0 it means that we first time record information
 *  to a file and on first iteration it put to
 *  the file info about dimensions of boundary box
 */
void owHelper::loadConfigurationFromFile(float *& position, float *& connections, int *& membranes, owConfigProperty * config, int iteration){
	try{
		if(iteration == 0)
			positionFile.open("./buffers/position_buffer.txt");
		int i = 0;
		float x, y, z, p_type;
		if( positionFile.is_open() )
		{
			if(iteration == 0){
				positionFile >> config->xmin;
				positionFile >> config->xmax;
				positionFile >> config->ymin;
				positionFile >> config->ymax;
				positionFile >> config->zmin;
				positionFile >> config->zmax;
				positionFile >> numOfElasticP;
				positionFile >> numOfLiquidP;
				config->setParticleCount(numOfElasticP + numOfLiquidP);
				position = new float[4 * config->getParticleCount()];
			}
			while( positionFile.good() &&  i < config->getParticleCount())
			{
				positionFile >> x >> y >> z >> p_type;
				position[i * 4 + 0] = x;
				position[i * 4 + 1] = y;
				position[i * 4 + 2] = z;
				position[i * 4 + 3] = p_type;
				i++;
			}
		}
		if(!positionFile.good()){
			positionFile.close();
			exit(0);
		}
		if(iteration == 0){

			ifstream connectionFile("./buffers/connection_buffer.txt");
			connections = new float[MAX_NEIGHBOR_COUNT * numOfElasticP * 4];
			if( connectionFile.is_open() )
			{
				int i = 0;
				float jd, rij0, val1, val2;
				while(connectionFile.good() && i < MAX_NEIGHBOR_COUNT * numOfElasticP){
					connectionFile >> jd >> rij0 >> val1 >> val2;
					connections[ 4 * i + 0 ] = jd;
					connections[ 4 * i + 1 ] = rij0;
					connections[ 4 * i + 2 ] = val1;
					connections[ 4 * i + 3 ] = val2;
					i++;
				}
			}
			connectionFile.close();
			ifstream membranesFile("./buffers/membranes_buffer.txt");
			if(membranesFile.is_open()){
				int m_count = 0;
				//membranesFile >> m_count;
				int i = 0;
				membranes = new int[4 * m_count];
				while(membranesFile.good() && i < m_count){
					membranesFile >> membranes[4 * i + 0] >> membranes[4 * i + 1] >> membranes[4 * i + 2] >> membranes[4 * i + 3];
					i++;
				}
			}
			membranesFile.close();
		}
	}catch(std::exception &e){
		std::cout << "ERROR_06: " << e.what() << std::endl;
		exit( -6 );
	}
}
/** Print value of elapsed time from last handling to watch_report method.
 *
 *  This function is required for logging time consumption info.
 *
 *  @param str
 *  represents output string format.
 */
void owHelper::watch_report( const char * str )
{
#if defined(_WIN32) || defined(_WIN64)
	QueryPerformanceCounter(&t2);
	printf(str,(t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart);
	t1 = t2;
	elapsedTime = (t2.QuadPart - t0.QuadPart) * 1000.0 / frequency.QuadPart;
#elif defined(__linux__)
	clock_gettime(CLOCK_MONOTONIC_RAW, &t2);
	time_t sec = t2.tv_sec - t1.tv_sec;
	long nsec;
	if (t2.tv_nsec >= t1.tv_nsec) {
			nsec = t2.tv_nsec - t1.tv_nsec;
	} else {
			nsec = 1000000000 - (t1.tv_nsec - t2.tv_nsec);
			sec -= 1;
	}
	printf(str,(float)sec * 1000.f + (float)nsec/1000000.f);
	t1 = t2;
	elapsedTime =  (float)(t2.tv_sec - t0.tv_sec) * 1000.f + (float)(t2.tv_nsec - t0.tv_nsec)/1000000.f;
#elif defined(__APPLE__)
    uint64_t elapsedNano;
    static mach_timebase_info_data_t    sTimebaseInfo;

    if ( sTimebaseInfo.denom == 0 ) {
        (void) mach_timebase_info(&sTimebaseInfo);
    }

    t2 = mach_absolute_time();
    elapsedNano = (t2-t1) * sTimebaseInfo.numer / sTimebaseInfo.denom;
    printf(str, (float)elapsedNano/1000000.f );
    t1=t2;
    elapsedNano = (t2-t0) * sTimebaseInfo.numer / sTimebaseInfo.denom;
    elapsedTime = (float)elapsedNano/1000000.f;
#endif
}
