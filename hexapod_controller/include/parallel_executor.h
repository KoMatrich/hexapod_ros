#pragma once
// parallel executor
// Copyright (c) 2024, Martin Kocich
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the Martin Kocich nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL MARTIN KOCICH BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Martin Kocich

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>

class ThreadPool {
public:
    ThreadPool(size_t num_threads)
    : stop(false) {
        for (size_t i = 0; i < num_threads; ++i) {
            workers.emplace_back(
                [this] {
                    while (true) {
                        // null target task
                        std::function<void()> task;

                        {
                            std::unique_lock<std::mutex> lock(this->queue_mutex);
                            // wait for task or end of ThreadPool
                            this->condition.wait(lock, [this] { return this->stop || !this->tasks.empty(); });

                            // check if for end
                            if ( this->stop )
                                return;

                            // set target task
                            task = std::move(this->tasks.front());
                            this->tasks.pop();
                        }

                        // execute task
                        task();
                    }
                }
            );
        }
    }

    // add task to queue
    template<class F, class... Args>
    auto inline enqueue(F&& f, Args&&... args)
        -> std::future<typename std::result_of<F(Args...)>::type> {

        // shortcut for return type
        using return_type = typename std::result_of<F(Args...)>::type;

        auto task = std::make_shared< std::packaged_task<return_type()> >(
            std::bind(f, args...)
        );

        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex);

            // Don't allow enqueueing after stopping the pool
            if (stop)
                throw std::runtime_error("enqueue on stopped ThreadPool");

            tasks.emplace([task]() { (*task)(); });
        }

        condition.notify_one();
        return res;
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread &worker : workers)
            worker.join();
    }

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;

    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};

// Enum class for execution modes
enum class ExecutionMode {
    SEQUENTIAL, // Execute one by one
    SYNC,       // Wait for all to finish before proceeding
    ASYNC,      // Execute all asynchronously, wait only if needed
};

template<typename T>
class ParallelExecutor {
public:
    ParallelExecutor(std::vector<T>& instances, ExecutionMode mode = ExecutionMode::ASYNC):
    instances(instances),
    MODE(mode),
    pool(std::thread::hardware_concurrency())
    {}

    ~ParallelExecutor() {
        // ensure all tasks are completed
        await();
    }

    // Execute class method in predefined mode using perfect forwarding
    template<typename Function, typename... Args>
    void inline execute(Function function, Args&&... args) {
        switch(MODE) {
            case ExecutionMode::SEQUENTIAL:
                runSeque(function, args...);
                break;
            case ExecutionMode::SYNC:
                runAsync(function, args...);
                await();
                break;
            case ExecutionMode::ASYNC:
                await();
                runAsync(function, args...);
                break;
        }
    }

    // Run class method sequentially
    template<typename Function, typename... Args>
    void inline runSeque(Function function, Args&&... args) {
        for (T& instance : instances) {
            (instance.*function)(args...);
        }
    }

    // Run class method asynchronously
    template<typename Function, typename... Args>
    void inline runAsync(Function function, Args&&... args) {
        for (T& instance : instances) {
            futures.push_back(pool.enqueue(
                [&, function] {
                    (instance.*function)(args...);
                }
            ));
        }
    }

    // Wait for all futures to finish
    void inline await() {
        for (auto& f : futures) {
            if (f.valid()) {
                f.wait();
            }
        }
        futures.clear(); // Clearing futures after completion
    }

private:
    std::vector<T>& instances;              // Instances to run on

    ExecutionMode MODE;                     // Execution mode
    std::vector<std::future<void>> futures; // Vector for managing async tasks
    ThreadPool pool;                        //
};
