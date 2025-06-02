#include <mutex>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <iostream>
#include <unistd.h>

#ifdef DEBUG
#define DEBUG_PRINT(s) std::cout<<s<<std::endl;
#else
#define DEBUG_PRINT(s) do {} while (0)
#endif


// Definitions of mutex, thread and condition variable types depending on OS
#ifdef __XENO__
	typedef RT_MUTEX mutex_t;
	typedef RT_TASK thread_t;
	typedef RT_COND condition_variable_t;
#else  // __XENO__
	// Standard C++ (non-RT)
	typedef std::mutex mutex_t;
	typedef std::thread thread_t;
	typedef std::condition_variable_any condition_variable_t;
#endif // __XENO__


// Platform specific mutex and condition variable handling

bool acquireMutex(mutex_t& mutex, int timeoutUs);

void releaseMutex(mutex_t& mutex);