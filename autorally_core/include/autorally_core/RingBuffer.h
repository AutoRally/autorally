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
 * @file RingBuffer.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 17, 2013
 * @copyright 2012 Georgia Institute of Technology
 * @brief RingBuffer class definition
 *
 ***********************************************/
#ifndef TIME_TAGGED_RING_BUFFER_H_
#define TIME_TAGGED_RING_BUFFER_H_

#include <ros/ros.h>
#include <ros/time.h>

#include <boost/circular_buffer.hpp>

namespace autorally_core
{
/**
 *  @class RingBuffer RingBuffer.h
 *  "autorally_core/RingBuffer.h"
 *  @brief
 *
 *
 */
template <class T>
class RingBuffer
{
 public:
  /**
   * @brief Constructor for RingBuffer
   *
   */
  RingBuffer();
  ~RingBuffer();

  bool update(std::pair<double, T> &newData);
  double maxKey() const;
  double minKey() const;
  bool interpolateKey(const double value, T &interpolated) const;
  bool interpolateValue(const double key, T &interpolated) const;
  T linearInterp(const T &first,
                 const T &second,
                 const double& distance) const;
  int size() const {return m_ringBuffer.size();}
  void clear() {m_ringBuffer.clear();}

 private:
  boost::circular_buffer<std::pair<double,T> > m_ringBuffer;
};

}
#endif //TIME_TAGGED_RING_BUFFER_H_
