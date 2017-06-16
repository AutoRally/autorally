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
 * @file DiagnosticsEntry.hpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date October 18, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief
 *
 * @details
 ***********************************************/
#ifndef DIAGNOSTICS_ENTRY_HPP_
#define DIAGNOSTICS_ENTRY_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/time.h>
#endif

#include <string>

#include <QtGui/QStandardItem>
#include <QtCore/QList>
#include <diagnostic_msgs/DiagnosticArray.h>

/**
 *  @class DiagnosticsEntry DiagnosticsEntry.hpp "ocs/DiagnosticsEntry.hpp"
 *  @brief
 */
class DiagnosticsEntry : public QObject {
  Q_OBJECT
public:

  /**
  * @brief Constructor, just initializes internal variables
  *
  */
  DiagnosticsEntry();
  virtual ~DiagnosticsEntry();


  void update(const diagnostic_msgs::DiagnosticArray& msg);


  QStandardItemModel* model() {return &m_model;}


  void setDiagnosticFrequency(const double diagFreq) {m_diagnosticFrequency = diagFreq;}

  /**
  * @brief Generates new entry for a sender (node) of diagnostics messages
  * @param level the message level to covnert to a color
  * @return QBrush the translated color (ERROR=red, WARN=yellow, OK=green)
  */
  QBrush colorFromLevel(unsigned char level);

public slots:
  /**
  * @brief Slot Indicates that an item in the diagnostics view was double clicked
  * @param index The location in the model that was clicked
  */
  void diagModelDoubleClicked(const QModelIndex& index);

  /**
  * @brief Remove all stale diagnostic messages
  */
  void clearStaleDiag();

  /**
  * @brief Update the time since last message received for all elements in OCS
  */
  void updateTimes();

private:
  enum DataIndexes { NAMECOL = 0,
                     HWIDCOL = 1,
                     MSGCOL = 2,
                     COUNTCOL = 3};
  enum KeyValIndexes { KEYCOL = 0,
                       VALCOL = 1,
                       TIMECOL = 2};

  QStandardItemModel m_model; ///<Model holding diagnostic messages
  double m_diagnosticFrequency;

  /**
  * @brief Generates new entry for a sender (node) of diagnostics messages
  */
  void newSender(const diagnostic_msgs::DiagnosticStatus& msg);

  /**
  * @brief Generates new entry for a diagnostics message from a sender
  */
  void newKeyValue(QStandardItem* parent, const diagnostic_msgs::KeyValue& msg);
  void updateKeyValue(QStandardItem* diagMsg,
                      const diagnostic_msgs::KeyValue& data);

  /**
  * @brief Determines the highest priority message comign from a sender
  * @param item the list of all messages from a sender
  * @return QBrush the highest priority message (ERROR > WARN > OK)
  */
  QBrush highestPriorityColor(QStandardItem* item);
};

#endif /* DIAGNOSTICS_ENTRY_HPP_ */
