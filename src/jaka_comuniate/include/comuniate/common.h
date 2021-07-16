#ifndef COMMON_H
#define COMMON_H

#include <semaphore.h>
#include <thread>
#include <atomic>
#include <mutex>

#define UNIXSOCKETNAME "/tmp/stream_socket"

class Uncopyable
{
protected:
     Uncopyable() {}
     ~Uncopyable() {}
private:
     Uncopyable(const Uncopyable&);
     Uncopyable& operator=(const Uncopyable&);
};

#endif // COMMON_H
