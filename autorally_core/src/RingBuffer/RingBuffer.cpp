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
 * @file RingBuffer.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 17, 2013
 * @copyright 2012 Georgia Institute of Technology
 * @brief RingBuffer class implementation
 *
 ***********************************************/
#include <autorally_core/RingBuffer.h>

namespace autorally_core
{

template <class T>
RingBuffer<T>::RingBuffer():
  m_ringBuffer(50)
{
}

template <class T>
RingBuffer<T>::~RingBuffer()
{
}

template <class T>
bool RingBuffer<T>::update(std::pair<double, T> &newData)
{
  if(m_ringBuffer.size() == 0)
  {
    m_ringBuffer.push_back(newData);
    return true;
  }

  typename boost::circular_buffer<std::pair<double, T> >::iterator it;
  for(it = m_ringBuffer.begin(); it != m_ringBuffer.end(); it++)
  {
    if(newData.first < it->first)
    {
      m_ringBuffer.insert(it+1, newData);
      return true;
    } else if(newData.first == it->first)//already have entry from that time
    {
      return false;
    }
  }
  m_ringBuffer.push_back(newData);
  return true;
}

template <class T>
double RingBuffer<T>::maxKey() const
{
  return m_ringBuffer.back().first;
}

template <class T>
double RingBuffer<T>::minKey() const
{
  return m_ringBuffer.front().first;
}

template <class T>
bool RingBuffer<T>::interpolateKey(const double value, T &interpolated) const
{

  //do not perform extrapolation
  if(value < m_ringBuffer.front().second || value > m_ringBuffer.back().second)
  {
    return false;
  }

//  std::cout << "Comparing:" << value << " to:";
  typedef typename boost::circular_buffer<std::pair<double, T> > buff;
  for(typename buff::const_iterator buffIt = m_ringBuffer.begin()+1;
      buffIt != m_ringBuffer.end();
      buffIt++)
  {
//    std::cout << buffIt->second << " ";
    if(buffIt->second == value)
    {
      interpolated = buffIt->first;
//      std::cout << std::endl;
      return true;
    }else if(buffIt->second > value)
    {
      interpolated = linearInterp(
                (buffIt-1)->first,                            
                buffIt->first,
                (value-(buffIt-1)->second)/(buffIt->second-(buffIt-1)->second));
      
      //std::cout << "Interpolating key between(" << buffIt->first << ":" << (buffIt-1)->first <<
      //             ")  for value" << value << std::endl;
      return true;
    }
  }
  std::cout << std::endl;

  return false;
}

template <class T>
bool RingBuffer<T>::interpolateValue(const double key, T &interpolated) const
{

  //do not perform extrapolation
  if(key < m_ringBuffer.front().first || key > m_ringBuffer.back().first)
  {
    return false;
  }

  //std::cout << "Comparing:" << key << " to:";
  typedef typename boost::circular_buffer<std::pair<double, T> > buff;
  for(typename buff::const_iterator buffIt = m_ringBuffer.begin()+1;
      buffIt != m_ringBuffer.end();
      buffIt++)
  {
    //std::cout << buffIt->first << " ";
    if(buffIt->first == key)
    {
      interpolated = buffIt->second;
      //std::cout << std::endl;
      return true;
    }else if(buffIt->first > key)
    {
      interpolated = linearInterp(
                          (buffIt-1)->second,
                          buffIt->second,
                          (key-(buffIt-1)->first)/(buffIt->first-(buffIt-1)->first));
      //std::cout << std::endl;
      return true;
    }
  }
  std::cout << std::endl;

  return false;
}

template <class T>
T RingBuffer<T>::linearInterp(const T &first, const T &second, const double& distance) const
{
  T interp;
  interp = first+distance*(second-first);
  return interp;
}

template <>
double RingBuffer<double>::linearInterp(const double& first, const double& second, const double& distance) const
{
  double interp;
  interp = first+distance*(second-first);
  return interp;
}

template class RingBuffer<double>;
template class RingBuffer<float>;
template class RingBuffer<int>;
}
