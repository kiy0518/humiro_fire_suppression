#ifndef THREAD_SAFE_QUEUE_H
#define THREAD_SAFE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>

template<typename T>
class ThreadSafeQueue {
public:
    ThreadSafeQueue(size_t max_size = 10) : max_size_(max_size) {}
    
    void push(const T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_) {
            queue_.pop();  // 오래된 항목 제거
        }
        queue_.push(item);
        condition_.notify_one();
    }
    
    bool try_pop(T& item, int timeout_ms = 0) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (timeout_ms > 0) {
            if (!condition_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                     [this] { return !queue_.empty(); })) {
                return false;  // 타임아웃
            }
        } else {
            condition_.wait(lock, [this] { return !queue_.empty(); });
        }
        
        if (queue_.empty()) {
            return false;
        }
        
        item = queue_.front();
        queue_.pop();
        return true;
    }
    
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            queue_.pop();
        }
    }
    
private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::queue<T> queue_;
    size_t max_size_;
};

#endif // THREAD_SAFE_QUEUE_H
