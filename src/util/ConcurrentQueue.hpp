/*
 * Copyright (c) 2013-2014, Michael Grey and Markus Theil
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <mutex>
#include <queue>
#include <condition_variable>

// http://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html

template <typename Data>
class ConcurrentQueue {
   private:
    std::queue<Data> the_queue;
    mutable std::mutex the_mutex;
    std::condition_variable the_condition_variable;

   public:
    void push(Data const& data) {
        std::unique_lock<std::mutex> lock(the_mutex);
        the_queue.push(data);
        lock.unlock();
        the_condition_variable.notify_one();
    }

    void push(Data const&& data) {
        std::unique_lock<std::mutex> lock(the_mutex);
        the_queue.push(data);
        lock.unlock();
        the_condition_variable.notify_one();
    }

    bool empty() const {
        std::unique_lock<std::mutex> lock(the_mutex);
        return the_queue.empty();
    }

    bool try_pop(Data& popped_value) {
        std::unique_lock<std::mutex> lock(the_mutex);
        if (the_queue.empty()) {
            return false;
        }

        popped_value = the_queue.front();
        the_queue.pop();
        return true;
    }

    void wait_and_pop(Data& popped_value) {
        std::unique_lock<std::mutex> lock(the_mutex);
        while (the_queue.empty()) {
            the_condition_variable.wait(lock);
        }

        popped_value = the_queue.front();
        the_queue.pop();
    }
};
