/**
 * @file   BlockingQueue.h
 * @brief  Blocking queue used to parallelize feature detection and stereo matching
 * @brief  Automatically implements wait and notify when work is available or when queue is full
 * @author Chris Beall
 */

#pragma once

#include <queue>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/optional.hpp>

template<typename WorkType>
class BlockingQueue {

private:
  size_t max_size_;
  std::queue<WorkType> queue_;
  boost::mutex mutex_;
  boost::condition_variable work_available_;
  boost::condition_variable slot_available_;

public:

  /// sets the maximum size of the blocking queue
  BlockingQueue(size_t max_size=10) : max_size_(max_size) {}

  //push work if queue is not full, waits for the notification from pop function if queue is full
  bool pushBlocking(const WorkType& element) {
    boost::mutex::scoped_lock guard(mutex_);

    //checks and waits if the queue is full
    //this thread would wait until a pop function is called and it notifies that a slot is available
    if (queue_.size() >= max_size_) {
      slot_available_.wait(guard);
    }

    //store work and notify one pop function that some work is available
    queue_.push(element);
    work_available_.notify_one();
    return true;
  }

  /// Push work if the queue is not full
  bool pushNonBlocking(const WorkType& element) {
    boost::mutex::scoped_lock guard(mutex_);

    // return false is queue is full and no more elements can be inserted
    if (queue_.size() >= max_size_) {
      return false;
    }
    queue_.push(element);

    // notifies one pop function that there is work available
    work_available_.notify_one();
    return true;
  }

  /// pops elements from the queue when work is available, waits if no work is available
  WorkType popBlocking() {
    boost::mutex::scoped_lock guard(mutex_);
    while (queue_.empty() == true) {
      //puts the thread on wait if there is no work available
      //this thread would wait until a push function is called and it notifies that work is available
      work_available_.wait(guard);
    }

    WorkType element = queue_.front();
    queue_.pop();

    //notifies one push thread that there is a slot available to store future work
    slot_available_.notify_one();
    return element;
  }

  /// pops elements from the queue when work is available, returns nothing otherwise
  boost::optional<WorkType> popNonBlocking() {
    boost::mutex::scoped_lock guard(mutex_);
    if (queue_.empty() == true) {
      return boost::optional<WorkType>();
    }

    WorkType element = queue_.front();
    queue_.pop();

    //notifies one push thread that there is a slot available to store future work
    slot_available_.notify_one();
    return boost::optional<WorkType>(element);
  }

  WorkType front()
  {
    boost::mutex::scoped_lock guard(mutex_);
    return queue_.front();
  }
  WorkType back()
  {
    boost::mutex::scoped_lock guard(mutex_);
    return queue_.back();
  }

  size_t size() const { return queue_.size(); }
  size_t max_size() const { return max_size_; }

};
