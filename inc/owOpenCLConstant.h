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

#ifndef OW_OPENCL_CONSTANT_H
#define OW_OPENCL_CONSTANT_H

#define MAX_NEIGHBOR_COUNT 32

#define MAX_MEMBRANES_INCLUDING_SAME_PARTICLE 7

#define LIQUID_PARTICLE 1
#define ELASTIC_PARTICLE 2
#define BOUNDARY_PARTICLE 3

#define NO_PARTICLE_ID -1
#define NO_CELL_ID -1
#define NO_DISTANCE -1.0f

#define QUEUE_EACH_KERNEL 1

#define INTEL_OPENCL_DEBUG 0

const int local_NDRange_size = 256;

enum DEVICE { CPU = 0, GPU = 1};
enum INTEGRATOR { EULER = 0, LEAPFROG = 1 };

#endif // #ifndef OW_OPENCL_CONSTANT_H
