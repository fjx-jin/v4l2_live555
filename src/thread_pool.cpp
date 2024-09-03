#include "thread_pool.h"
#include <stdio.h>

ThreadPool* ThreadPool::m_instance = new ThreadPool(50,1000);

ThreadPool::ThreadPool(int min,int max) : m_minThreadNums(min),
m_maxThreadNums(max)
{
    m_stop= false;
    m_exitThreads = 0;
    m_idleThreads = m_curThreads = min;
    m_manager = new thread(&ThreadPool::manager,this);
    for(int i = 0;i<m_curThreads;i++)
    {
        thread t(&ThreadPool::worker,this);
        m_workers.insert(make_pair(t.get_id(),move(t)));
    }
}

ThreadPool::~ThreadPool()
{
    m_stop = true;
    m_cond.notify_all();
    for(auto& it : m_workers)
    {
        thread& t = it.second;
        if(t.joinable())
        {
            t.join();
        }
    }

    if(m_manager->joinable())
        m_manager->join();
    
    delete m_manager;
}

// void ThreadPool::addTask(callback f)
// {
//     {
//         lock_guard<mutex> lock(m_queueMutex);
//         m_tasks.emplace(f);
//     }
//     m_cond.notify_one();
// }

void ThreadPool::manager()
{
    while(!m_stop.load())
    {
        this_thread::sleep_for(chrono::seconds(2));
        int idle = m_idleThreads.load();
        int current  = m_curThreads.load();
        if (idle > current / 2 && current > m_minThreadNums)
        {
            //空闲太多 计划消除几个线程
            m_exitThreads.store(2);
            m_cond.notify_all();
            unique_lock<mutex> lock(m_idMutex);
            for(const auto& id : m_idsToClean)
            {
                auto it = m_workers.find(id);
                if(it != m_workers.end())
                {
                    printf("因空闲线程太多 %d exit.\n",*(uint32_t*)&(*it).first);
                    (*it).second.join();
                    m_workers.erase(it);
                }
            }
            m_idsToClean.clear();
        }
        else if(idle == 0 && current < m_maxThreadNums)
        {
            thread t(&ThreadPool::worker,this);
            m_workers.insert(make_pair(t.get_id(),move(t)));
            m_curThreads++;
            m_idleThreads++;
        }
    }
}

void ThreadPool::worker()
{
    while(!m_stop.load())
    {
        callback task = nullptr;
        {
            unique_lock<mutex> lock(m_queueMutex);
            while(!m_stop.load() && m_tasks.empty())
            {
                m_cond.wait(lock);
                if (m_exitThreads.load() > 0)
                {
                    // printf("因空闲线程太多 %d exit.\n",);
                    m_exitThreads--;
                    m_curThreads--;
                    unique_lock<mutex> lck(m_idMutex);
                    m_idsToClean.emplace_back(this_thread::get_id());
                    return;
                }
            }

            if(!m_tasks.empty())
            {
                //取出一个任务执行
                task=move(m_tasks.front());
                m_tasks.pop();
            }
        }

        if(task)
        {
            m_idleThreads--;
            task();
            m_idleThreads++;
        }
    }
}

// int calc(int x, int y)
// {
//     int res = x + y;
//     //cout << "res = " << res << endl;
//     this_thread::sleep_for(chrono::seconds(2));
//     return res;
// }

// int main()
// {
//     ThreadPool pool(4);
//     vector<future<int>> results;

//     for (int i = 0; i < 10; ++i)
//     {
//         results.emplace_back(pool.addTask(calc, i, i * 2));
//     }

//     // 等待并打印结果
//     for (auto&& res : results)
//     {
//         printf("线程函数返回值: %d.\n",res.get());
//     }

//     return 0;
// }