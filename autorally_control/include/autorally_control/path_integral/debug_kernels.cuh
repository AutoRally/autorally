/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file debug_kernels.cuh
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Header file for debug kernel launch functions.
 ***********************************************/

#ifndef MPPI_DEBUG_KERNELS_CUH_
#define MPPI_DEBUG_KERNELS_CUH_

namespace autorally_control {

__global__ void debugCostKernel(float x, float y, float heading, int width_m ,int height_m, int ppm,
                                cudaTextureObject_t tex, float* debug_data_d, float3 c1, 
                                float3 c2, float3 trs)
{
  //Get the thread indices
  int x_idx = blockIdx.x*blockDim.x + threadIdx.x;
  int y_idx = blockIdx.y*blockDim.y + threadIdx.y;
  //Convert these into meters
  float x_pos = x_idx/(1.0*ppm);
  float y_pos = y_idx/(1.0*ppm);
  x_pos -= width_m/2.0;
  y_pos -= height_m/2.0;
  x_pos += x; //x position in world coordinates
  y_pos += y; //y position in world coordinates
  float u,v,w;
  //Compute a projective transform of (x, y, 0, 1)
  u = c1.x*x_pos + c2.x*y_pos + trs.x;
  v = c1.y*x_pos + c2.y*y_pos + trs.y;
  w = c1.z*x_pos + c2.z*y_pos + trs.z;
  //Compute the cost for the current position
  float cost = tex2D<float4>(tex, u/w, v/w).x;
  //Write the cost to the debug data array
  if (x_idx < width_m*ppm && (height_m*ppm - y_idx) < height_m*ppm) {
    float x_transformed = cosf(heading)*(x_pos - x) + sinf(heading)*(y_pos - y);
    float y_transformed = -sinf(heading)*(x_pos - x) + cosf(heading)*(y_pos - y);
    float dist = 0.25*fabs(x_transformed) + fabs(y_transformed);
    if ( dist < .15 && x_transformed > 0){
      if ( dist < .1 && x_transformed > 0.05){
        cost = 1;
      } else{
        cost = 0;
     }
    }
    int idx = (height_m*ppm - (y_idx+1))*(width_m*ppm) + x_idx;
    if ((idx > 0) && (idx < (width_m*ppm)*(height_m*ppm)) ){
      debug_data_d[idx] = cost;
    }
  }
}

void launchDebugCostKernel(float x, float y, float heading, int width_m, int height_m, int ppm,
                           cudaTextureObject_t tex, float* debug_data_d, float3 c1,
                           float3 c2, float3 trs, cudaStream_t stream)
{
  dim3 dimBlock(16, 16, 1);
  dim3 dimGrid((width_m*ppm - 1)/16 + 1, (height_m*ppm - 1)/16 + 1, 1);
  debugCostKernel<<<dimGrid, dimBlock, 0, stream>>>(x, y, heading, width_m, height_m, ppm, tex, debug_data_d,
                                         c1, c2, trs);
  HANDLE_ERROR( cudaStreamSynchronize(stream) );
}

};

#endif /*MPPI_DEBUG_KERNELS_CUH_*/
