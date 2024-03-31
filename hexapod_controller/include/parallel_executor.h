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
#include <thread>
#include <future>

enum class ExecutionMode {
    SEQUENTIAL, // one by one
    SYNC,       // wait for all to finish
    ASYNC,      // wait only if needed
};

template<typename T>
class ParallelExecutor {
public:
    ParallelExecutor(std::vector<T>& instances, ExecutionMode mode=ExecutionMode::ASYNC)
    : MODE(mode),
    instances(instances)
    {}

    ~ParallelExecutor() {
        await();
    }

    // execute class method in predefined mode
    template<typename Function, typename... Args>
    inline void execute(Function function, Args... args) {
        switch(MODE){
            case ExecutionMode::SEQUENTIAL:
                runSeque(function,args...);
                break;
            case ExecutionMode::SYNC:
                runAsync(function,args...);
                await();
                break;
            case ExecutionMode::ASYNC:
                await();
                runAsync(function,args...);
                break;
        }
    }

    // run class method sequential
    template<typename Function, typename... Args>
    inline void runSeque(Function function, Args... args) {
        for (T& instance : instances) {
            (instance.*function)(args...);
        }
    }

    // run class method async
    template<typename Function, typename... Args>
    inline void runAsync(Function function, Args... args){
        for (T& instance : instances) {
            std::packaged_task<void()> task([=, &instance] { (instance.*function)(args...); });
            futures.emplace_back(task.get_future());
            std::thread(std::move(task)).detach();
        }
    }

    // wait for all futures to finish
    inline void await(){
        for (auto& f : futures) {
            if (f.valid()) {
                f.wait();
            }
        }
        futures.clear();
    }

private:
    ExecutionMode MODE;                     // execution mode
    std::vector<T>& instances;              // instances run on
    std::vector<std::future<void>> futures; // futures for async
};
