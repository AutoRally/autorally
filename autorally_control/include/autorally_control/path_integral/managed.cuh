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
 * @file managed.cuh
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Class to be inherited by classes passed 
 * by reference to CUDA kernels. ALL classes passed to MPPI
 * as costs or dynamics need to inherit from this class
 ***********************************************/

#ifndef MPPI_MANAGED_CUH_
#define MPPI_MANAGED_CUH_

#include <stdio.h>

class Managed 
{
public:

  cudaStream_t stream_ = 0;

  /*void *operator new(size_t len) {
    void *ptr;
    cudaMallocManaged(&ptr, len, cudaMemAttachHost);
    cudaDeviceSynchronize();
    return ptr;
  }

  void operator delete(void *ptr) {
    cudaDeviceSynchronize();
    cudaFree(ptr);
  }*/

  void bindToStream(cudaStream_t stream) {
    stream_ = stream;
    cudaDeviceSynchronize();
    /*if (stream_ == 0){
      cudaStreamAttachMemAsync(stream_, this, 0, cudaMemAttachGlobal);
    }
    else {
      cudaStreamAttachMemAsync(stream_, this, 0, cudaMemAttachSingle);
    }
    cudaDeviceSynchronize();
    */
  }

};


#endif /* MPPI_MANAGED_CUH_*/