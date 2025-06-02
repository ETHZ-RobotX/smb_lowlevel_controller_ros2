#include "roboteq_controller/inter_process_communication.h"

bool acquireMutex(mutex_t &mutex, int timeoutUs) {
    //DEBUG_PRINT("Trying to acquire mutex with timeout of: "<<timeoutUs<<" microseconds");
    if (timeoutUs >= 0) {
        if (mutex.try_lock()) { return true; }

        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

        while (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start) < std::chrono::microseconds(timeoutUs)) {
            if (mutex.try_lock()) { return true; }
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    } else {
        mutex.lock();
        return true;
    }
    return false;
}

void releaseMutex(mutex_t &mutex) {
    mutex.unlock();
}