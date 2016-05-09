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
/**
 * @file ImageMaskEntry.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date October 18, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief
 **/
#include "ImageMaskEntry.hpp"

ImageMaskEntry::ImageMaskEntry()
{
  m_model.setColumnCount(2);
}

ImageMaskEntry::~ImageMaskEntry() {
}

void ImageMaskEntry::newEntry(const autorally_msgs::imageMask& mask)
{
  QStandardItem* newEntry = new QStandardItem(mask.sender.c_str());
  newEntry->setEditable(false);
  newEntry->setCheckable(true);
  newEntry->setCheckState(Qt::Checked);
  newEntry->setBackground(generateColor());

  newEntry->setChild(PROW,NAMECOL, new QStandardItem("Points:"));
  newEntry->setChild(PROW,DATACOL,
                       new QStandardItem(QString::number(mask.points.size()) ));
  newEntry->setChild(LROW,NAMECOL, new QStandardItem("Lines:"));
  newEntry->setChild(LROW,DATACOL,
                       new QStandardItem(QString::number(mask.lines.size()) ));
  newEntry->setChild(RROW,NAMECOL, new QStandardItem("ROIs:"));
  newEntry->setChild(RROW,DATACOL,
                       new QStandardItem(QString::number(mask.rois.size())));
  m_model.appendRow(newEntry);
}

bool ImageMaskEntry::update(const autorally_msgs::imageMask& mask)
{
  QList<QStandardItem *> items = m_model.findItems(mask.sender.c_str());
  if(items.empty())
  {
    newEntry(mask);
    m_masks.insert(std::pair<std::string, autorally_msgs::imageMask>(mask.sender, mask));

  } else if(items.size() == 1)
  {
    m_masks[mask.sender] = mask;
    items[0]->child(PROW,DATACOL)->setText(QString::number(mask.points.size()));
    items[0]->child(LROW,DATACOL)->setText(QString::number(mask.lines.size()));
    items[0]->child(RROW,DATACOL)->setText(QString::number(mask.rois.size()));
  } else
  {
    ROS_ERROR("Something is wrong with the imageMaskModel!");
  }
  return true;
}

const autorally_msgs::imageMask& ImageMaskEntry::mask(const std::string& maskName) const
{
  return m_masks.find(maskName)->second;
}

const QStandardItem* ImageMaskEntry::itemFromName(const std::string& itemName) const
{
  QList<QStandardItem *> items = m_model.findItems(itemName.c_str());
  if(items.size() == 1)
  {
    return items[0];
  }
  return NULL;
}

const QColor ImageMaskEntry::generateColor()
{
  //pick a random color from
  //{Qt::red, Qt::green, Qt::blue, Qt::cyan, Qt::magenta, Qt::yellow}
  int c = rand()%6;
  return Qt::GlobalColor(7+c);
}
