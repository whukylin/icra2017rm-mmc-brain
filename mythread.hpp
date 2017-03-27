#pragma once

#include <pthread.h>

class MyThread
{
public:
	MyThread();
	virtual ~MyThread();
	int create(void*(*func)(void*), void* param);
	int join();
	int detach();
	int exit();
private:
	pthread_t tid;
};

