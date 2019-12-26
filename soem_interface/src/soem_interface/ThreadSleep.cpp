#include <soem_interface/common/ThreadSleep.hpp>

#include <thread>

namespace soem_interface {

void threadSleep(const double duration) { std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1e9 * duration))); }

}  // namespace soem_interface
