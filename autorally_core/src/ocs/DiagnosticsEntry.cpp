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
 * @file DiagnosticsEntry.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date October 18, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief
 **/
#include "DiagnosticsEntry.hpp"

DiagnosticsEntry::DiagnosticsEntry()
{
	QStringList header;
  header << "Name" << "Hardware ID" << "Message" << "Count";
	m_model.setColumnCount(4);
  m_model.setHorizontalHeaderLabels(header);
}

DiagnosticsEntry::~DiagnosticsEntry() {

}

void DiagnosticsEntry::newSender(const diagnostic_msgs::DiagnosticStatus& msg)
{
  QList<QStandardItem*> newSender;
  newSender << new QStandardItem(msg.name.c_str());
  newSender << new QStandardItem(msg.hardware_id.c_str());
  newSender << new QStandardItem(msg.message.c_str());
  newSender << new QStandardItem("0");

  newSender[NAMECOL]->setEditable(false);
  newSender[HWIDCOL]->setEditable(false);
  newSender[MSGCOL]->setEditable(false);
  newSender[COUNTCOL]->setEditable(false);
  m_model.appendRow(newSender);
}

void DiagnosticsEntry::newKeyValue(QStandardItem* parent, const diagnostic_msgs::KeyValue& msg)
{
  QList<QStandardItem*> newMessage;
  newMessage << new QStandardItem(msg.key.c_str());

  QBrush color = colorFromLevel(msg.value[0]);
  if(color == Qt::white)
  {
    newMessage << new QStandardItem(msg.value.c_str());
  } else
  {
    newMessage << new QStandardItem("0.000");
    newMessage[VALCOL]->setData(QVariant(ros::Time::now().toSec()));
    newMessage[KEYCOL]->setBackground(color);
  }
  //newMessage << new QStandardItem("1");
  newMessage << new QStandardItem();
  newMessage << new QStandardItem();

  newMessage[KEYCOL]->setEditable(false);
  newMessage[VALCOL]->setEditable(false);
  newMessage[TIMECOL]->setEditable(false);
  newMessage[COUNTCOL]->setEditable(false);

  parent->appendRow(newMessage);
}

void DiagnosticsEntry::update(const diagnostic_msgs::DiagnosticArray& msg)
{
  //go through each DiagnosticStatus in the array
  std::vector<diagnostic_msgs::DiagnosticStatus>::const_iterator diagIt;
  for(diagIt = msg.status.begin(); diagIt != msg.status.end(); diagIt++)
  {
    //ROS_INFO("Looking at %s", diagIt->name.c_str());
    //find the entry in the diagnostics model, otherwise add a new one
    QList<QStandardItem *> items = m_model.findItems(diagIt->name.c_str());
    if(items.isEmpty() && diagIt->message != "Node starting up")
    {
      //add new item for the sender
      newSender(*diagIt);
      items = m_model.findItems( diagIt->name.c_str() );
    }
    
    if(items.size() == 1)
    {
      if(diagIt->hardware_id == m_model.item(items.front()->index().row(),1)->text().toStdString())
      {
        m_model.item(items.front()->index().row(),2)->setText(diagIt->message.c_str());
        items.front()->setData(diagIt->level);

        for(const auto value : diagIt->values)
        {
          updateKeyValue(items.front(), value);
        }
      }
      else
      {
        newSender(*diagIt);
        update(msg);
      }
    }
    else if(!diagIt->hardware_id.empty())
    {
      bool foundMatch = false;
      for( auto item : items)
      {
        if(diagIt->hardware_id == m_model.item(item->index().row(),1)->text().toStdString())
        {
          foundMatch = true;
          m_model.item(item->index().row(),2)->setText(diagIt->message.c_str());
          item->setData(diagIt->level);

          for(const auto value : diagIt->values)
          {
            updateKeyValue(item, value);
          }
        }
      }
      if(!foundMatch)
      {
        newSender(*diagIt);
        update(msg);
      }
    } else {
      ROS_WARN_STREAM("DiagnosticsEntry: No hardware ID in Diagnostics message: " << diagIt->name << " but multiple diagnostic entries present.");
    }
  }
}

void DiagnosticsEntry::updateKeyValue(QStandardItem* diagMsg,
                                      const diagnostic_msgs::KeyValue& data)
{
  QBrush color;
  //int count;
  double time;
  bool found = false;
  //bool ok = false;
  //QString num;

  //update the message just received
  for(int i = 0; i < diagMsg->rowCount(); i++)
  {
    if(diagMsg->child(i)->text().toStdString() == data.key)
    {
      //ROS_INFO("%s -%s-", data.key.c_str(), data.value.c_str());
      found = true;
      if(data.value.empty())
      {
        color = QBrush(Qt::white);
      } else
      {
        color = colorFromLevel(data.value[0]);
      }

      //if its a normal diagnostic message set the text to the value,
      //otherwise it is an auto_rally diagnostic that has a level associated
      if(color == Qt::white)
      {
        diagMsg->child(i,1)->setText(data.value.c_str());
      } else
      {
        diagMsg->child(i,0)->setBackground(color);
        time = ros::Time::now().toSec();
        diagMsg->child(i,1)->setData(QVariant(time));
      }

      //count = diagMsg->child(i,2)->text().toInt(&ok) + 1;
      //ROS_INFO("count %d", count);

      //if(ok && diagMsg->child(i,2))
      //{
        //num.setNum(count);
        //ROS_INFO("%s", diagMsg->child(i,2)->text().toStdString().c_str());
        //ROS_INFO("num [%s]", QString::number(count).toStdString().c_str());
        //diagMsg->child(i,2)->setText(QString::number(count));
        //ROS_INFO("after");
      //} else
      //{
      //  ROS_ERROR("Bad conversion");
      //}
      //ROS_INFO("end count %d", count);
    }
  }
  //ROS_INFO("updateKeyValue halfway");
  //if it is a new message from the sender, add it
  if(!found)
  {
    newKeyValue(diagMsg, data);
    //set the number of messages for the sender
    m_model.item(diagMsg->index().row(),3)->
      setText(QString::number(diagMsg->rowCount()));
  }
  diagMsg->setBackground(highestPriorityColor(diagMsg));
  //ROS_INFO("updateKeyValue done");
}

QBrush DiagnosticsEntry::colorFromLevel(unsigned char level)
{
  switch(level)
  {
    case 0 :
      return Qt::green;
    case 1 :
      return Qt::yellow;
    case 2 :
      return Qt::red;
    default :
      return Qt::white;
  }
}

QBrush DiagnosticsEntry::highestPriorityColor(QStandardItem* item)
{
  bool foundGreen = false;
  bool foundYellow = false;

  QBrush toReturn(Qt::white);
  if(item->data().isValid())
  {
    toReturn = colorFromLevel(item->data().toChar().toLatin1());
  }
  else if(!item->hasChildren())
  {
    return QBrush(Qt::white);
  }

  for(int i = 0; i < item->rowCount(); i++)
  {

    if(item->child(i)->background().color() == Qt::red ||
       toReturn.color() == Qt::red)
    {
      return QBrush(Qt::red);
    }

    if(item->child(i)->background().color() == Qt::yellow ||
       toReturn.color() == Qt::yellow)
    {
      foundYellow = true;
    } else if(item->child(i)->background().color() == Qt::green)
    {
      foundGreen = true;
    }
  }

  if(foundYellow)
  {
    toReturn = Qt::yellow;
  } else if(foundGreen)
  {
    toReturn = Qt::green;
  }

  return toReturn;
}

void DiagnosticsEntry::diagModelDoubleClicked(const QModelIndex& index)
{
  if(index.parent().isValid())
  {
    //remove the row that was clicked on
    QStandardItem* parent = m_model.itemFromIndex(index.parent());
    parent->removeRow(index.row());
    parent->setBackground(highestPriorityColor(parent));

    //set number of messages for parent entry
    QStandardItem* entry = m_model.item(parent->index().row(),3);
    entry->setText(QString::number(parent->rowCount()));
  }
}

void DiagnosticsEntry::clearStaleDiag()
{
  QStandardItem* node;

  //go through each nodes diagnostics messages
  for(int i = 0; i < m_model.rowCount(); i++)
  {
    node = m_model.item(i);

    //go through each message
    for(int j = 0; j < node->rowCount();)
    {
      if(node->child(j,1)->background() == Qt::magenta)
      {
        //remove that entry by index
        diagModelDoubleClicked(node->child(j,1)->index());
      } else
      {
        /*only increment the index if there was nothing removed, otherwise
          the second of two stale messages in a row will be skipped
        */
        j++;
      }
    }
  }
}

void DiagnosticsEntry::updateTimes()
{
  double time;
  QStandardItem* node;
  QStandardItem* child1;
  bool anyStale = false;

  for(int i = 0; i < m_model.rowCount(); i++)
  {
    if( (node = m_model.item(i)) == 0)
    {
      ROS_ERROR("Invalid Parent Node");
    } else
    {
      anyStale = false;
      for(int j = 0; j < node->rowCount(); j++)
      {
        //only update the times if the model has time data associated with it
        //this allows non auto_rally diagnostic messages to be not colored
        if( (child1 = node->child(j,1)) != 0 &&
            node->child(j,2) != 0 &&
            child1->data().isValid())
        {
          time = ros::Time::now().toSec()-child1->data().toDouble();

          //consider message stale if a new one has not been received for a while,
          //color part of it magenta as well as part of the parent
          if(time > 5*m_diagnosticFrequency)
          {
            child1->setBackground(Qt::magenta);
            anyStale = true;
          }
          else
          {
            child1->setBackground(node->child(j,2)->background());
          }

          child1->setText(QString::number(time, 'g', 4));
        } else //there are no key-value pairs with priority, reset bkgnd color
        {
          m_model.item(i,1)->setBackground(m_model.item(i,2)->background());
        }
      }

      if(anyStale)
      {
        m_model.item(i,1)->setBackground(Qt::magenta);
      } else
      {
        m_model.item(i,1)->setBackground(m_model.item(i,2)->background());
      }
    }
  }
}
