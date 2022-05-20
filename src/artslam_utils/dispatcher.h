#ifndef ARTSLAM_CORE_DISPATCHER_H
#define ARTSLAM_CORE_DISPATCHER_H

#include <functional>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>

namespace artslam::core::utils {
    class Dispatcher {
    public:
        // Defines the configuration parameters of the class
        struct Configuration {
            bool verbose_ = true;
        };

        // Defines a type representing a generic method, returning void as long as the arguments are pre-bound
        typedef std::function<void()> fp_t;

        // Class constructor, with parameters:
        // - name of the dispatcher
        // - number of threads used to execute operations
        explicit Dispatcher(std::string name, size_t thread_count = 1);

        // Class constructor, with parameters:
        // - configuration of the dispatcher
        // - name of the dispatcher
        // - number of threads used to execute operations
        Dispatcher(const Configuration& configuration, std::string name, size_t thread_count = 1);

        // Class destructor
        ~Dispatcher();

        // Adds a constant operation to the queue of operations to execute (dispatch and copy)
        void dispatch(const fp_t& op);

        // Adds an operation to the queue of operations to execute (dispatch and move)
        void dispatch(fp_t&& op);

        // Deletes the copy constructor
        Dispatcher(const Dispatcher& rhs) = delete;

        // Deletes the copy assignment operator
        Dispatcher& operator = (const Dispatcher& rhs) = delete;

        // Deletes the move constructor
        Dispatcher(Dispatcher&& rhs) = delete;

        // Deletes the move assignment operator
        Dispatcher& operator = (Dispatcher&& rhs) = delete;

    private:
        // Handles the threads, assigning operations to execute, depending on their availability
        void dispatch_thread_handler();

        // ----------------------------------------------------------------------------------
        // ---------------------------- PARAMETERS AND VARIABLES ----------------------------
        // ----------------------------------------------------------------------------------
        bool verbose_;                          // whether the class should be verbose
        std::string name_;                      // name of the dispatcher
        std::mutex lock_;                       // lock used to handle the queue of operations
        std::vector<std::thread> threads_;      // vector of threads used to execute different operations
        std::queue<fp_t> op_queue_;             // queue of operations to execute
        std::condition_variable cv_;            // condition variable used to wake or stop threads
        bool quit_;                             // whether the threads should stop processing operations
    };
}


#endif //ARTSLAM_CORE_DISPATCHER_H
