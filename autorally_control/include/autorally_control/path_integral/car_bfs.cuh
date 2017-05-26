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
 * @file car_bfs.cuh
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Class for non-linear basis function model.
 ***********************************************/

#ifndef CAR_BFS_CUH_
#define CAR_BFS_CUH_

#include "managed.cuh"

namespace autorally_control {

class CarBasisFuncs : public Managed
{
public:
  __host__ __device__ float basisFuncX(int idx, float* s, float* u)
  {
    float phi = 0;
    switch(idx) {
      case 0: phi = u[1];
              break;
      case 1: phi = s[4]/10.0;
              break;
      case 2: phi = (s[4] > .1) ? 
                    sinf(u[0])*tanf(atanf(s[5]/s[4] + .45*s[6]/s[4]) - u[0])/1200.0: 
                    sinf(u[0])*tanf(-u[0])/1200.0;
              break;
      case 3: phi = (s[4] > .1) ? 
                    sinf(u[0])*tanf(atanf(s[5]/s[4] + .45*s[6]/s[4]) - u[0])*fabs(tanf(atanf(s[5]/s[4] + .45*s[6]/s[4]) - u[0]))/1440000.0:
                    sinf(u[0])*tanf(-u[0])*fabs(tanf(-u[0]))/1440000.0;
              break;            
      case 4: phi = (s[4] > .1) ?
                    sinf(u[0])*powf(tanf(atanf(s[5]/s[4] + .45*s[6]/s[4]) - u[0]), 3)/1728000000.0 :
                    sinf(u[0])*powf(tanf(-u[0]), 3)/1728000000.0;
              break;
      case 5: phi = s[6]*s[5]/25.0;
              break;
      case 6: phi = s[6]/10.0;
              break;
      case 7: phi = s[5]/10.0;
              break;
      case 8: phi = sinf(u[0]);
              break;
      case 9: phi = (s[4] > .1) ? 
                     s[5]/s[4]/40.0 : 
                     0;
              break;
      case 10: phi = (s[4] > .1) ? 
                      tanf(atanf(s[5]/s[4] + .45*s[6]/s[4]) - u[0])/1400.0 : 
                      tanf(-u[0])/1400.0;
              break;
      case 11: phi = (s[4] > .1) ? 
                      tanf(atanf(s[5]/s[4] + .45*s[6]/s[4]) - u[0])*fabs(tanf(atanf(s[5]/s[4] + .45*s[6]/s[4]) - u[0]))/1960000:
                      tanf(-u[0])*fabs(tanf(-u[0]))/1960000;
              break;
      case 12: phi = (s[4] > .1) ?
                      powf(tanf(atanf(s[5]/s[4] + .45*s[6]/s[4]) - u[0]), 3)/2744000000:
                      powf(tanf(-u[0]), 3)/2744000000;
              break;
      case 13: phi = (s[4] > .1) ? 
                     (s[5]/s[4] - .35*s[6]/s[4])/40.0 : 
                      0;
              break;
      case 14: phi = (s[4] > .1) ? 
                     (s[5]/s[4] - .35*s[6]/s[4])*fabs(s[5]/s[4] - .35*s[6]/s[4])/1600.0: 
                      0;
              break;            
      case 15: phi = (s[4] > .1) ? 
                      powf(s[5]/s[4] - .35*s[6]/s[4], 3)/64000.0 : 
                      0;
              break;
      case 16: phi = s[6]*s[4]/50.0;
              break;
      case 17: phi = s[3];
              break;
      case 18: phi = s[3]*s[6];
              break;
      case 19: phi = s[3]*s[4]/3.0;
              break;
      case 20: phi = s[3]*s[4]*s[6]/5.0;
              break;
      case 21: phi = powf(s[4], 2)/100.0; //s[2]/5.0;
              break;
      case 22: phi = powf(s[4], 3)/1000.0; //s[2]*s[3];
              break;
      case 23: phi = powf(u[1], 2);
              break;
      case 24: phi = powf(u[1], 3);
              break;
    }
    return phi;
  }
};

}

#endif