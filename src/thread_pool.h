#ifndef _THREAD_POOL_H_
#define _THREAD_POOL_H_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <queue>
#include <map>
#include <future>
#include <functional>

using namespace std;
typedef function<void()> callback;
class ThreadPool
{
public:
    static ThreadPool* instance() {return m_instance;}
    ~ThreadPool();
    template<typename F, typename... Args>
    auto addTask(F&& f, Args&&... args) -> future<typename result_of<F(Args...)>::type>
    {
        using returnType = typename result_of<F(Args...)>::type;
        auto task = make_shared<packaged_task<returnType()>>(
            bind(forward<F>(f), forward<Args>(args)...)
        );
        future<returnType> res = task->get_future();
        {
            unique_lock<mutex> lock(m_queueMutex);
            m_tasks.emplace([task]() { (*task)(); });
        }
        m_cond.notify_one();
        return res;
    }

private:
    ThreadPool(int min= 4,int max = 256);
    void manager();
    void worker();

private:
    static ThreadPool* m_instance;
    thread* m_manager;
    map<thread::id, thread> m_workers; 
    vector<thread::id> m_idsToClean; 
    int m_minThreadNums;
    int m_maxThreadNums;
    atomic<bool> m_stop;
    atomic<int> m_curThreads;
    atomic<int> m_idleThreads;
    atomic<int> m_exitThreads;
    queue<callback> m_tasks;
    mutex m_idMutex;
    mutex m_queueMutex;
    condition_variable m_cond;
};

#endif