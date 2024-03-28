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

// template<typename T>
// class ParallelExecutor {
// public:
//     ParallelExecutor() {}
//     ~ParallelExecutor() {
//         for (auto& f : futures) {
//             if (f.valid()) {
//                 f.wait();
//             }
//         }
//     }

//     template<typename Function, typename... Args>
//     void run(std::vector<T>& instances, Function function, Args... args) {
//         // Wait for all previous tasks to complete
//         for (auto& f : futures) {
//             if (f.valid()) {
//                 f.wait();
//             }
//         }
//         futures.clear();

//         // Launch a thread for each instance
//         for (T& instance : instances) {
//             std::packaged_task<void()> task([=, &instance] { (instance.*function)(args...); });
//             futures.emplace_back(task.get_future());
//             std::thread(std::move(task)).detach();
//         }
//     }
    
// private:
//     std::vector<std::future<void>> futures;
// };

template<typename T>
class ParallelExecutor {
public:
    ParallelExecutor() {}

    template<typename Function, typename... Args>
    void run(std::vector<T>& instances, Function function, Args... args) {
        std::vector<std::future<void>> futures;

        // Wait for all tasks to complete
        for (auto& f : futures) {
            if (f.valid()) {
                f.wait();
            }
        }

        // Launch a thread for each instance
        for (T& instance : instances) {
            std::packaged_task<void()> task([=, &instance] { (instance.*function)(args...); });
            futures.emplace_back(task.get_future());
            std::thread(std::move(task)).detach();
        }
    }
};
