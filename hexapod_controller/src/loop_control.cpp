
// Loop speed control
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

#include <loop_control.h>

LoopControl::LoopControl(uint rate, const std::string name):
    RATE(rate), NAME(name), loop_rate(rate)
{
    zero();
}

bool LoopControl::sleep(){
    bool fast_enough = loop_rate.sleep();

    // Check if loop is fast enough
    if( fast_enough ){
        return true;
    }

    // Handling for case when loop is not fast enough

    double cycle_time = loop_rate.cycleTime().toNSec();
    cycle_time /= 1000000; // convert nS to mS

    if( max_cycle_time < cycle_time )
        max_cycle_time = cycle_time;

    if( min_cycle_time > cycle_time )
        min_cycle_time = cycle_time;

    cant_keep_up_counter++;
    cumulative_cycle_time += cycle_time;

    if( cant_keep_up_counter >= RATE ){

        double average_cycle_time = cumulative_cycle_time / cant_keep_up_counter;

        ROS_WARN(
            "%s is running %0.1f/%dHz Min:%0.1fms. A:%0.1fms. Max:%0.1fms.",
             NAME.c_str(), 1000.0 / average_cycle_time, RATE, min_cycle_time, average_cycle_time, max_cycle_time
        );

        zero();

        // Prevent feedback loop caused by printing
        loop_rate.reset();
    }

    return false;
}

void LoopControl::zero(){
    cant_keep_up_counter = 0;
    cumulative_cycle_time = 0;

    max_cycle_time = 0;
    min_cycle_time = DBL_MAX;
}
