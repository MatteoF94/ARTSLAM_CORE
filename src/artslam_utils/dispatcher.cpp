#include "dispatcher.h"

#include <utility>
#include <sstream>
#include <iostream>

using namespace artslam::core::utils;

// Class constructor, with parameters
Dispatcher::Dispatcher(std::string name, size_t thread_count) : name_(std::move(name)), threads_(thread_count){
    quit_ = false;
    verbose_ = true;

    std::stringstream msg;
    msg << "[Dispatcher] Created dispatcher with (name, #threads): (" << name_ << ", " << thread_count << ")\n";
    std::cout << msg.str();

    for(auto & thread : threads_) {
        thread = std::thread(&Dispatcher::dispatch_thread_handler, this);
    }
}

// Class constructor, with parameters
Dispatcher::Dispatcher(const Configuration& configuration, std::string name, size_t thread_count) : name_(std::move(name)), threads_(thread_count){
    quit_ = false;
    verbose_ = configuration.verbose_;

    if(verbose_) {
        std::stringstream msg;
        msg << "[Dispatcher] Created dispatcher with (name, #threads): (" << name_ << ", " << thread_count << ")\n";
        std::cout << msg.str();
    }

    for(auto & thread : threads_) {
        thread = std::thread(&Dispatcher::dispatch_thread_handler, this);
    }
}

// Class destructor
Dispatcher::~Dispatcher() {
    std::stringstream msg;
    if(verbose_) {
        msg << "[Dispatcher] Destroying dispatcher with (name, #threads): (" << name_ << ", " << threads_.size()
            << ")\n";
        std::cout << msg.str();
    }

    std::unique_lock<std::mutex> lock(lock_);
    quit_ = true;
    lock.unlock();
    cv_.notify_all();

    for(auto & thread : threads_) {
        if(thread.joinable()) {
            thread.join();
        }
    }

    if(verbose_) {
        msg.str("");
        msg << "[Dispatcher] Finished destroying dispatcher with (name, #threads): (" << name_ << ", "
            << threads_.size() << ")\n";
        std::cout << msg.str();
    }
}

// Adds a constant operation to the queue of operations to execute (dispatch and copy)
void Dispatcher::dispatch(const fp_t &op) {
    std::unique_lock<std::mutex> lock(lock_);
    op_queue_.push(op);
    lock.unlock();      // notice that the unlocking is done manually, before the notification
    cv_.notify_one();
}

// Adds an operation to the queue of operations to execute (dispatch and move)
void Dispatcher::dispatch(fp_t &&op) {
    std::unique_lock<std::mutex> lock(lock_);
    op_queue_.push(std::move(op));
    lock.unlock();      // notice that the unlocking is done manually, before the notification
    cv_.notify_one();
}

// Handles the threads, assigning operations to execute, depending on their availability
void Dispatcher::dispatch_thread_handler() {
    std::unique_lock<std::mutex> lock(lock_);

    do {
        // wait until data is available or a quit signal
        cv_.wait(lock, [this]{return (!op_queue_.empty() || quit_);});

        // after waiting, the lock is re-acquired
        if(!quit_ && !op_queue_.empty()) {
            auto op = std::move(op_queue_.front());
            op_queue_.pop();

            // unlock after handling the queue
            lock.unlock();

            // execute the operation
            op();

            // reacquire lock before the next cycle
            lock.lock();
        }
    } while(!quit_);
}