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

#ifndef OW_PHYSICS_CONSTANT_H
#define OW_PHYSICS_CONSTANT_H

#include <math.h>

#include "owOpenCLConstant.h"
// Main physical constants contain 

#define generateBodyConfiguration 1 //or load from file otherwise [0/1]

#ifndef M_PI
#define M_PI 3.1415927f
#endif


const float rho0 = 1000.0f;                         // Standard value of liquid density for water (kg/m^3)

const float mass = 1e-10f;                          // Mass for one particle (kg).

const float timeStep = 1.0e-05f;                    // Time step of simulation (s)
                                                    // NOTE: "For numerical stability and convergence, several time step
                                                    // constraints must be satisfied. The Courant-Friedrich-Levy
                                                    // (CFL) condition for SPH (dt <= lambda_v*(h/v_max))
                                                    // states that the speed of numerical propagation must be
                                                    // higher than the speed of physical propagation, where v_max = max(||v_i(t)||)
                                                    // is the maximum magnitude of the velocity throughout the simulation. lambda_v is a constant factor, e. g.
                                                    // lambda_v = 0.4. In other words, a particle i must not move more than its smoothing length h in one time step. Fur-
                                                    // thermore, high accelerations might influence the simulation
                                                    // results negatively. Therefore, the time step must also satisfy
                                                    // dt <= lambda_f*sqrt(h/F_max)
                                                    // where F_max=max(||F_i(t)||) denotes the magnitude of
                                                    // the maximum force per unit mass for all particles throughout
                                                    // the simulation. lambda_f = 0.25."
                                                    // For more info [1, page 5].
                                                    // NOTE: actually it depends on mass too for bigger value of mass it possible
                                                    // to use bigger value of time step. Dependence on mass could be described by following
                                                    // mass influent on simulation scale and due to the fact that we're simulating incompressible
                                                    // liquid start configuration of particles should satisfy condition that density in
                                                    // every particle of configuration <= rh0. So if we decrease mass we should decrease
                                                    // distance among particles than it leads to that we should decrease time step.
                                                    // TODO: find dependence and make choice automatically
                                                    // [1] M. Ihmsen, N. Akinci, M. Gissler, M. Teschner, Boundary Handling and Adaptive Time-stepping for PCISPH Proc. VRIPHYS, Copenhagen, Denmark, pp. 79-88, Nov 11-12, 2010.
                                                    // ATTENTION! too large values can lead to 'explosion' of elastic matter objects

const float simulationScale = 1.14f*0.0037f*pow(mass,1.f/3.f)/pow(0.00025f,1.f/3.f);//pow(mass,1.f/3.f)/pow(rho0,1.f/3.f); // Simulation scale coefficient. It means that N * simulationScale
                                                                   // converts from simulation scale to meters N / simulationScale convert from meters simulation scale
                                                                   // If you want to take real value of distance in meters you need multiple on simulation scale
                                                                   // NOTE: simulationScale depends from mass of particle. If we place one particle
                                                                   // into volume with some size of side we want that density in this value is equal to rho0

const float h = 3.34f;                              // Smoothed radius value. This is dimensionless invariant parameter.
                                                    // For taken real value in meter you need multiple this on simulationScale.
                                                    // h is a spatial distance, over which their properties are "smoothed" by a kernel function [1].
                                                    // [1] https://en.wikipedia.org/wiki/Smoothed-particle_hydrodynamics

const float hashGridCellSize = 2.0f * h;            // All bounding box is divided on small spatial cells with size of side == h. Size of side for one spatial cell
                                                    // This require for spatial hashing and => searching a neighbors
const float r0 = 0.5f * h;                          // Standard distance between two boundary particle == equilibrium distance between 2 particles [1]
                                                    // [1] M. Ihmsen, N. Akinci, M. Gissler, M. Teschner, Boundary Handling and Adaptive Time-stepping for PCISPH Proc. VRIPHYS, Copenhagen, Denmark, pp. 79-88, Nov 11-12, 2010.

const float viscosity = 0.000302f;                   // liquid viscosity value //why this value? Dynamic viscosity of water at 25 C = 0.89e-3 Pa*s
const double beta = timeStep*timeStep*mass*mass*2/(rho0*rho0); // B. Solenthaler's dissertation, formula 3.6 (end of page 30)

const double Wpoly6Coefficient = 315.0 / ( 64.0 * M_PI * pow( (double)(h*simulationScale), 9.0 ) ); // Wpoly6Coefficient for kernel Wpoly6 [1]
                                                                                                    // [1] Solenthaler (Dissertation) page 17 eq. (2.20)

const double gradWspikyCoefficient= -45.0 / ( M_PI * pow( (double)(h*simulationScale), 6.0 ) );     // gradWspikyCoefficient for kernel gradWspiky [1]
                                                                                                    // [1] Solenthaler (Dissertation) page 18 eq. (2.21)

const double divgradWviscosityCoefficient = - gradWspikyCoefficient;                                // divgradWviscosityCoefficient for kernel Viscous [1]
                                                                                                    // [1] Solenthaler (Dissertation) page 18 eq. (2.22)
/* We' re using Cartesian coordinate system
				y|
				 |___x
				 /
			   z/
 */
const float gravity_x =  0.0f;                      // Value of vector Gravity component x
const float gravity_y = -9.8f;                      // Value of vector Gravity component y
const float gravity_z =  0.0f;                      // Value of vector Gravity component z
const int maxIteration = 3;                         // Max number of iterations for Predictive-Corrective scheme

const float mass_mult_Wpoly6Coefficient = (float) ( (double)mass * Wpoly6Coefficient );                       // Conversion of double value to float. For work with only 1st precision arithmetic.
const float mass_mult_gradWspikyCoefficient = (float) ( (double)mass * gradWspikyCoefficient );               // It needs for work with devices don't support double precision.
const float mass_mult_divgradWviscosityCoefficient = (float) ( (double)mass * divgradWviscosityCoefficient ); // Also it helps to increase performance and memory consumption

const float hashGridCellSizeInv = 1.0f / hashGridCellSize; // Inverted value for hashGridCellSize
const float simulationScaleInv = 1.0f / simulationScale;   // Inverted value for simulationScale
const float _hScaled = h * simulationScale;         // scaled smoothing radius
const float _hScaled2 = _hScaled*_hScaled;          // squared scaled smoothing radius
const float surfTensCoeff = mass_mult_Wpoly6Coefficient * simulationScale;
const float elasticityCoefficient = 0.18f / mass; // Elasticity coefficient

#endif // #ifndef OW_PHYSICS_CONSTANT_H
