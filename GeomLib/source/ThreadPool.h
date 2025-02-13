#pragma once
#include "Timer.h"
#include <condition_variable>
#include <functional>
#include <vector>
#include <thread>
#include <atomic>
#include <future>
#include <queue>
#include <set>

namespace geomlib
{
	class ThreadTask
	{
	protected:
		int Id;
		virtual void ToDo() = 0;
		void RunTask() { ToDo(); }
		friend class ThreadPool;

	public:
		ThreadTask() : Id(0) { }
	};

	class ThreadPool
	{
	private:
		std::queue<std::shared_ptr<ThreadTask>> qTasks;
		std::vector<std::thread> vecThreads;
		std::set<int> available;
		std::atomic<int> completed;
		std::condition_variable cvAssigner;
		std::condition_variable cvFinish;
		bool end = false;
		int taskIndex = 0;
		std::mutex mtxCompleted;
		std::mutex mtxAvailable;
		std::mutex mtxIndex;
		std::mutex mtxQueue;
		std::atomic<bool> stop;
		friend class ThreadTask;

		void Run()
		{
			while (!stop) {
				std::unique_lock<std::mutex> qLock(mtxQueue);
				cvAssigner.wait(qLock, [this]()->bool { return !qTasks.empty() || stop; });
				if (!qTasks.empty())
				{
					std::shared_ptr<ThreadTask> task = std::move(qTasks.front());
					qTasks.pop();
					qLock.unlock();
					task->RunTask();

					completed++;
					cvFinish.notify_all();
				}
			}
		}

	public:
		ThreadPool(int threadNum = std::thread::hardware_concurrency())
		{
			for (int i = 0; i < threadNum; i++)
			{
				vecThreads.emplace_back(&ThreadPool::Run, this);
			}
		}

		void AssignTask(const std::shared_ptr<ThreadTask>& task)
		{
			int cur = taskIndex++;

			std::unique_lock<std::mutex> qLock(mtxQueue);
			qTasks.push(task);
			qLock.unlock();

			cvAssigner.notify_one();
		}

		void WaitEnd()
		{
			std::unique_lock<std::mutex> fin(mtxCompleted);
			cvFinish.wait(fin, [this]()->bool 
				{
					return qTasks.empty() && taskIndex == completed;
				});
		}

		~ThreadPool()
		{
			stop = true;
			cvAssigner.notify_all();
			for (int i = 0; i < vecThreads.size(); i++)
			{
				vecThreads[i].join();
			}
		}

	};
}