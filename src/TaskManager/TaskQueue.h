/*
 * taskQueue.h
 *
 *  Created on: 30/mag/2014
 *      Author: antlab
 */

#ifndef TASKQUEUE_H_
#define TASKQUEUE_H_

#include <queue>
#include "Tasks/Tasks.h"
#include "boost/thread/condition.hpp"

using namespace std;

class TaskQueue{


public:

	bool empty();
	void push(Task* t);
	Task* front();
	int size();
	void pop();
	void wait_for_data();

private:
	queue<Task*> task_queue;
	mutable boost::mutex the_mutex;
    boost::condition_variable task_queue_not_empty;

};

#endif /* TASKQUEUE_H_ */
