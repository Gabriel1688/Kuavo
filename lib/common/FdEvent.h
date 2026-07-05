/*
 * The Driver Station Library (LibDS)
 * Copyright (c) Lily Wang and other contributors.
 * Open Source Software; you can modify and/or share it under the terms of
 * the MIT license file in the root directory of this project.
 */
#pragma once

#include <functional>
#include <memory>
#include <pthread.h>
#include <queue>
#include <string>
#include <sys/eventfd.h>

using TCallback = std::function<void(std::string &result)>;

class FdEvent {
public:
    FdEvent();
    ~FdEvent();
    int fd() const { return fd_; }
    bool wait();
    bool notify();

private:
    int fd_;
};

template<typename T>
class Fifo {
public:
    Fifo() { pthread_mutex_init(&lock_, NULL); }

    ~Fifo() {
        pthread_mutex_destroy(&lock_);
    }

    int getFd() const { return fd_event_.fd(); }

    void push(std::unique_ptr<T> item) {
        pthread_mutex_lock(&lock_);
        queue_.push(std::move(item));
        pthread_mutex_unlock(&lock_);
        fd_event_.notify();
    }

    std::unique_ptr<T> pop() {
        std::unique_ptr<T> item = nullptr;
        fd_event_.wait();
        pthread_mutex_lock(&lock_);
        if (!queue_.empty()) {
            item = std::move(queue_.front());
            queue_.pop();
        }
        pthread_mutex_unlock(&lock_);
        return item;
    }

private:
    pthread_mutex_t lock_;
    std::queue<std::unique_ptr<T>> queue_;
    FdEvent fd_event_;
};
