#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/condition.hpp>
#include "TaskQueue.h"
#include "Tasks/Tasks.h"

class NodeManager;

class TaskManager{

public:

	TaskManager(NodeManager *nm);

	int sent_notifications;

//	void set_nodeManager(NodeManager *nm){
//		nodeManager_ptr = nm;
//	}
	void start(){
		m_thread = boost::thread(&TaskManager::taskManagerThread, this);
	}
	void join(){
		m_thread.join();
	}
	void addTask(Task *new_task);
	void deleteTask(Task *ended_task);

private:

	//std::queue<task*> task_queue;
	TaskQueue task_queue;
	NodeManager *nodeManager_ptr;
	boost::thread m_thread;
	void taskManagerThread();
	boost::mutex monitor;




};

#endif
