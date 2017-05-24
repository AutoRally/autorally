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
 * @file meta_math.h
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief template arithmetic for figuring out how
 * much memory to allocate for neural network on GPU
 ***********************************************/

#ifndef META_MATH_
#define META_MATH_

template<typename... Args>
constexpr int param_counter(int first) {
  return first;
}

template<typename... Args>
constexpr int param_counter(int first, int next) {
  return (first+1)*next;
}

template<typename... Args>
constexpr int param_counter(int first, int next, Args... args) {
  return (first+1)*next + param_counter(next, args...);
}

template<typename... Args>
constexpr int layer_counter(int first) {
  return 1;
}

template<typename... Args>
constexpr int layer_counter(int first, Args... args) {
  return 1 + layer_counter(args...);
}

template<typename... Args>
constexpr int neuron_counter(int first) {
  return first;
}

template<typename... Args>
constexpr int neuron_counter(int first, Args... args) {
  return (first > neuron_counter(args...)) ? first : neuron_counter(args...);
}

#endif /* META_MATH_ */