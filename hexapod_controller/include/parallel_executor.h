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

#include <iostream>
#include <vector>
#include <future>

// Enum class for execution modes
enum class ExecutionMode {
    SEQUENTIAL, // Execute one by one
    SYNC,       // Wait for all to finish before proceeding
    ASYNC,      // Execute all asynchronously, wait only if needed
};

template<typename T>
class ParallelExecutor {
public:
    ParallelExecutor(std::vector<T>& instances, ExecutionMode mode = ExecutionMode::ASYNC)
    : MODE(mode),
    instances(instances)
    {}

    ~ParallelExecutor() {
        // ensure all tasks are completed
        await();
    }

    // Execute class method in predefined mode using perfect forwarding
    template<typename Function, typename... Args>
    void execute(Function function, Args&&... args) {
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
    void runSeque(Function function, Args&&... args) {
        for (T& instance : instances) {
            (instance.*function)(args...);
        }
    }

    // Run class method asynchronously
    template<typename Function, typename... Args>
    void runAsync(Function function, Args&&... args) {
        for (T& instance : instances) {
            futures.push_back(std::async(std::launch::async, [=, &instance] { (instance.*function)(args...); }));
        }
    }

    // Wait for all futures to finish
    void await() {
        for (auto& f : futures) {
            if (f.valid()) {
                f.wait();
            }
        }
        futures.clear(); // Clearing futures after completion
    }

private:
    ExecutionMode MODE;                     // Execution mode
    std::vector<T>& instances;              // Instances to run on
    std::vector<std::future<void>> futures; // Futures for managing async tasks
};
