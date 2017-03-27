#include "mythread.hpp"

MyThread::MyThread()
{
	
}

MyThread::~MyThread()
{
}

int MyThread::create(void*(*func)(void*), void* param)
{
	return pthread_create(&tid, NULL, func, param); 
}

int MyThread:: join()
{
	return pthread_join(tid, NULL);
}
int MyThread::detach()
{
	return pthread_detach(tid);
}

int MyThread:: exit()
{
	pthread_exit(NULL);
}


