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
 * @file ImageMaskEntry.hpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date October 18, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief
 *
 * @details
 ***********************************************/
#ifndef IMAGE_MASK_ENTRY_HPP_
#define IMAGE_MASK_ENTRY_HPP_

#include <ros/ros.h>
#include <ros/time.h>
#include <string>

#include <QtGui/QStandardItem>
#include <QtCore/QList>
#include <autorally_msgs/imageMask.h>

/**
 *  @class ImageMaskEntry ImageMaskEntry.hpp "ocs/ImageMaskEntry.hpp"
 *  @brief
 */
class ImageMaskEntry {
public:

  std::map<std::string, autorally_msgs::imageMask> m_masks;

  /**
  * @brief Constructor, just initializes internal variables
  *
  */
  ImageMaskEntry();
  virtual ~ImageMaskEntry();
  bool update(const autorally_msgs::imageMask& mask);
  const autorally_msgs::imageMask& mask(const std::string &maskName) const;
  const QStandardItem* itemFromName(const std::string& itemName) const;
  QStandardItemModel* model() {return &m_model;}
  bool maskChecked();

private:
  enum DataIndexes { NAMECOL = 0,
                     DATACOL = 1,
                     PROW = 0,
                     LROW = 1,
                     RROW = 2 };

  QStandardItemModel m_model; ///<Model holding imageMask info

  void newEntry(const autorally_msgs::imageMask& mask);
  const QColor generateColor();
};

#endif /* IMAGE_MASK_ENTRY_HPP_ */
