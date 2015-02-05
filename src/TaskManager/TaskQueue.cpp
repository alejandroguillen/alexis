/*
 * taskQueue.cpp
 *
 *  Created on: 30/mag/2014
 *      Author: antlab
 */

#include "TaskManager/TaskQueue.h"

using namespace std;

	bool TaskQueue::empty()
	{
		boost::mutex::scoped_lock lock(the_mutex);
		return task_queue.empty();
	}

	void TaskQueue::push(Task* t){
		boost::mutex::scoped_lock lock(the_mutex);
		bool const was_empty = task_queue.empty();
		task_queue.push(t);
		if(was_empty){
			task_queue_not_empty.notify_one();
			cout << "TQ: notifying a new task was added" << endl;
		}
	}

	Task* TaskQueue::front(){
		boost::mutex::scoped_lock lock(the_mutex);
		return task_queue.front();
	}

	int TaskQueue::size(){
		boost::mutex::scoped_lock lock(the_mutex);
		return task_queue.size();
	}

	void TaskQueue::pop(){
		boost::mutex::scoped_lock lock(the_mutex);
		task_queue.pop();
	}

	void TaskQueue::wait_for_data(){
		boost::mutex::scoped_lock lock(the_mutex);
		while(task_queue.empty())
		{
			cout << "TQ: Waiting until a task is present" << endl;
			task_queue_not_empty.wait(lock);
			cout << "TQ: something woke me up!" << endl;
		}
	}

	
	void TaskQueue::wait_and_pop(Task* t){
			boost::mutex::scoped_lock lock(the_mutex);
			while(task_queue.empty())
			{
				task_queue_not_empty.wait(lock);
			}
			t = task_queue.front();
			task_queue.pop();
		}
		
	void TaskQueue::push2(Task* t)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        task_queue.push(t);
        lock.unlock();
        task_queue_not_empty.notify_one();
    }


    bool TaskQueue::try_pop(Task* t)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if(task_queue.empty())
        {
            return false;
        }
        
        t=task_queue.front();
        task_queue.pop();
        return true;
    }
