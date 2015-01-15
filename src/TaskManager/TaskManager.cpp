#include <stdlib.h>
#include <iostream>
#include "boost/thread/condition.hpp"
#include "TaskManager/TaskManager.h"
#include "NodeManager/NodeManager.h"
#include "Messages/Message.h"
#include "Tasks/Tasks.h"
#include "TaskManager/TaskQueue.h"

using namespace std;

TaskManager::TaskManager(NodeManager *nm){
	nodeManager_ptr = nm;
	sent_notifications = 0;
}
void TaskManager::addTask(Task *new_task){

	task_queue.push(new_task);
	//cout << "TM: adding a new task to the queue" << endl;

}

void TaskManager::deleteTask(Task *ended_task){
	delete(ended_task);
}

void TaskManager::taskManagerThread(){

	//cout << "TM: I'm entering the main thread" << endl;

	sent_notifications = 0;

	while(1){
		task_queue.wait_for_data();
		//cout << "TM: Now a task is present!" << endl;

		//de-queue the task and execute it
		Task *cur_task = task_queue.front();
		cout << (int)task_queue.size() << ", "  << task_queue.empty() << endl;
		task_queue.pop();
		cout << "TM: executing the task " << cur_task->type << endl;
		cout << (int)task_queue.empty() << endl;
		cur_task->execute();



		// notify the conclusion of the task
		sent_notifications++;
		nodeManager_ptr->notify_endTask();

	}


}
