#pragma once

#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>

namespace vanjee
{
namespace lidar
{
        template <typename T>
        class SyncQueue
        {
        private:
            std::queue<T> queue_;
            std::mutex mtx_;
#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY       
            std::condition_variable cv_; 
#endif

        public:
            inline size_t push(const T &value)
            {
#ifndef ENABLE_WAIT_IF_QUEUE_MEPTY
                bool empty = false;
#endif
                size_t size = 0;
                {
                    std::lock_guard<std::mutex> lg(mtx_);
#ifndef ENABLE_WAIT_IF_QUEUE_MEPTY
                    empty = queue_.empty();
#endif
                    queue_.push(value);
                    size = queue_.size();
                }
#ifndef ENABLE_WAIT_IF_QUEUE_MEPTY
                if (empty)
                {
                    cv_.notify_one();
                }
#endif
                return size;
            }
            inline T pop()
            {
                T value;
                std::lock_guard<std::mutex> lg(mtx_);
                if (!queue_.empty())
                {
                    value = queue_.front();
                    queue_.pop();
                }
                return value;
            }
            inline T popWait(unsigned int usec = 1000000)
            {
#ifdef ENABLE_WAIT_IF_QUEUE_MEPTY
                T value;
                {
                    std::lock_guard<std::mutex> lg(mtx_);
                    if (!queue_.empty())
                    {
                        value = queue_.front();
                        queue_.pop();
                        return value;
                    }
                }
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                return value;
#else
                T value;
                std::unique_lock<std::mutex> ul(mtx_);
                cv_.wait_for(ul, std::chrono::microseconds(usec), [this]
                             { return (!queue_.empty()); });
                if (!queue_.empty())
                {
                    value = queue_.front();
                    queue_.pop();
                }
                return value;
#endif
            }
            inline void clear()
            {
                std::queue<T> empty;
                std::lock_guard<std::mutex> lg(mtx_);
                swap(empty, queue_);
            }
        };

} 

} 
