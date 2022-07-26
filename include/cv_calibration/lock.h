#ifndef __LOCK_H__
#define __LOCK_H__

#include <atomic>
#include <mutex>

// Locks the thread_lock on constructor, releases on destructor
struct lock_t
{
    std::mutex& m;
    bool first = true; // For first run
    lock_t(std::mutex& value) : m(value)
    {
        // Lock the mutex
        value.lock();
    }
    
    operator bool()
    {
        // Return true on first go and then false
        bool r = first;
        first = false;
        return r;
    }

    ~lock_t()
    {
        // Unlock the mutex
        m.unlock();
    }
};

// Declares a "for" loop that only runs once, but the lock_t object only exists during the "for" loop scope
#define lock(tlock)\
for (lock_t _lok ## tlock{tlock};_lok ## tlock;)

#endif // __LOCK_H__