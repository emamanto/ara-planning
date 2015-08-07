#pragma once

#include <vector>
#include <utility>
#include <map>
#include <queue>
#include <set>
#include <iostream>
#include <unistd.h>
#include <ctime>
#include <boost/thread.hpp>

//#define FIRST_SOL
// #define MAZE_FIGURE
// #ifdef MAZE_FIGURE
// #undef FIRST_SOL
// #endif

template <typename S, typename P>
class search_node
{
public:
    search_node() {}
    search_node(S s) : state(s) {}

    bool operator < (const search_node<S, P>& other) const
    {return f_value > other.f_value; }
    bool operator > (const search_node<S, P>& other) const
    {return f_value < other.f_value; }

    S state;
    float f_value;
    std::vector<P> path;
};

template <typename S, typename P>
class search_result
{
public:
    std::vector<P> path;
    std::set<S> expanded;
    std::set<S> inconsistent;
};

// Info that needs to be thread safe
template <typename S, typename P>
class search_request
{
public:
    search_request(S start_pose,
                   std::vector<P> big,
                   std::vector<P> small,
                   float time_lim = -1,
                   bool hard_lim = false,
                   float eps = 10.f) :
        killed(false),
        paused(false),
        pause_waiting(false),
        time_limit(time_lim),
        start_time(0),
        hard_limit(hard_lim),
        start(start_pose),
        big_prs(big),
        s_prs(small),
        epsilon(eps) {};

    search_request() {};

    // Should really only call this when the search is no longer
    // going for this request...
    search_request& operator=(const search_request& other)
    {
        boost::lock_guard<boost::mutex> guard1(kill_mtx);
        boost::lock_guard<boost::mutex> guard2(solns_mtx);
        boost::lock_guard<boost::mutex> guard3(epsilon_mtx);
        killed = false;
        paused = false;
        solutions.clear();

        time_limit = other.time_limit;
        start_time = other.start_time;
        hard_limit = other.hard_limit;
        start = other.start;
        big_prs = other.big_prs;
        s_prs = other.s_prs;
        epsilon = other.epsilon;

        return *this;
    }

    void go()
    {
        boost::lock_guard<boost::mutex> guard1(kill_mtx);
        boost::lock_guard<boost::mutex> guard2(pause_mtx);
        start_time = std::clock();
    }

    S begin() const
    {
        return start;
    }

    void kill()
    {
        boost::lock_guard<boost::mutex> guard(kill_mtx);
        killed = true;
    }

    bool check_killed()
    {
        boost::lock_guard<boost::mutex> guard(kill_mtx);
        if (!killed && hard_limit && time_limit > 0)
        {
            if ((((float)(std::clock() - start_time)) /
                 CLOCKS_PER_SEC) > time_limit)
            {
                killed = true;
            }
        }
        return killed;
    }

    void pause()
    {
        pause_mtx.lock();
        paused = true;
        pause_mtx.unlock();
        set_pause_waiting(true);
        while(check_pause_waiting())
        {
            sleep(0.01);
        }
    }

    void set_pause_waiting(bool b)
    {
        boost::lock_guard<boost::mutex> guard(waiting_mtx);
        pause_waiting = b;
    }

    bool check_pause_waiting()
    {
        boost::lock_guard<boost::mutex> guard(waiting_mtx);
        return pause_waiting;
    }

    void unpause()
    {
        boost::lock_guard<boost::mutex> guard(pause_mtx);
        paused = false;
    }

    bool check_paused()
    {
        boost::lock_guard<boost::mutex> guard(pause_mtx);
        if (!paused && !hard_limit && time_limit > 0)
        {
            if ((((float)(std::clock() - start_time)) /
                 CLOCKS_PER_SEC) > time_limit)
            {
                paused = true;
            }
        }
        return paused;
    }

    void add_solution(search_result<S, P> addition)
    {
        boost::lock_guard<boost::mutex> guard(solns_mtx);
        solutions.push_back(addition);
    }

    std::vector<search_result<S, P> > copy_solutions()
    {
        boost::lock_guard<boost::mutex> guard(solns_mtx);
        return solutions;
    }

    void set_epsilon(float eps)
    {
        boost::lock_guard<boost::mutex> guard(epsilon_mtx);
        epsilon = eps;
    }

    float check_epsilon()
    {
        boost::lock_guard<boost::mutex> guard(epsilon_mtx);
        return epsilon;
    }

    // Copy
    std::vector<P> big_primitives() const
    {
        return big_prs;
    }

    // Copy
    std::vector<P> small_primitives() const
    {
        return s_prs;
    }

    int num_solutions()
    {
        boost::lock_guard<boost::mutex> guard(solns_mtx);
        return solutions.size();
    }

private:
    std::vector<search_result<S, P> > solutions;
    bool killed;
    bool paused;
    bool pause_waiting;
    float time_limit;
    std::clock_t start_time;
    bool hard_limit;
    S start;
    std::vector<P> big_prs;
    std::vector<P> s_prs;
    float epsilon;
    boost::mutex solns_mtx;
    boost::mutex kill_mtx;
    boost::mutex pause_mtx;
    boost::mutex waiting_mtx;
    boost::mutex epsilon_mtx;
};

template <class S>
float heuristic(S s) { return s.heuristic(); }

// NOT thread safe, only access within search thread while searching
template <typename S, typename P>
struct search_progress_info
{
    S goal;
    bool goal_found;
    std::vector<P> best_path;
    std::set<search_node<S, P> > INCONS;
    std::priority_queue<search_node<S, P> > OPEN;
    std::map<S, float> costs;
};

template <class S, typename P>
bool improve_path(search_request<S, P>& request,
                  search_progress_info<S, P>& progress)
{
    std::set<S> CLOSED = std::set<S>();

    // Copy out of thread-safe structure just once
    float epsilon = request.check_epsilon();
    std::vector<P> small_primitives = request.small_primitives();
    std::vector<P> big_primitives = request.big_primitives();

    search_result<S, P> result;
    result.path = progress.best_path;
    std::vector<P>* primitives;
    bool extra_prim = false;

    bool can_kill = !(request.num_solutions() == 0);

    while(!progress.goal_found ||
          (progress.costs[progress.goal] +
           epsilon*heuristic(progress.goal)) >
          progress.OPEN.top().f_value)
    {
        while (request.check_paused())
        {
            if (request.check_pause_waiting())
            {
                request.set_pause_waiting(false);
            }
            sleep(0.1);
        }

        if (can_kill && request.check_killed())
        {
            return false;
        }

        if (progress.OPEN.empty())
        {
            result.path.clear();
            request.add_solution(result);
            std::cout << "No solution found by improve_path"
                      << std::endl;
            return false;
        }
        // remove s with smallest fvalue from OPEN
        search_node<S, P> s(progress.OPEN.top());
        progress.OPEN.pop();
        result.expanded.insert(s.state);

        // CLOSED = CLOSED U s
        CLOSED.insert(s.state);

        if (s.state.small_steps())
        {
            primitives = &small_primitives;
            if (s.state.use_finisher())
            {
                primitives->push_back(s.state.compute_finisher());
                extra_prim = true;
            }
        }
        else
        {
            primitives = &big_primitives;
        }

        for (typename std::vector<P>::iterator p = primitives->begin();
             p != primitives->end(); p++)
        {
            S child = s.state.apply(*p);
            if (!child.valid()) continue;
            float new_cost = progress.costs[s.state] +
                s.state.cost(*p);

            // if s' not visited before (g(s') = inf)
            // or g(s') > g(s) + c(s, s')
            if ( !progress.costs.count(child) ||
                 progress.costs[child] > new_cost)
            {
                // g(s') = g(s) + c(s, s')
                progress.costs[child] = new_cost;

                search_node<S, P> nnode;
                nnode.state = child;
                nnode.path = s.path;
                nnode.path.push_back(*p);
                nnode.f_value = (progress.costs[child] +
                                 epsilon*heuristic(child));

                if (nnode.state.is_goal())
                {
                    result.path = nnode.path;
                    progress.goal = nnode.state;
                    progress.goal_found = true;
                }

                if (!CLOSED.count(child))
                {
                    // f(s') = g(s') + epsilon*heuristic(s')
                    progress.OPEN.push(nnode);
                }
                else
                {
                    progress.INCONS.insert(nnode);
                }
            }
        }
        if (extra_prim)
        {
            primitives->pop_back();
            extra_prim = false;
        }
    }

    if (result.path != progress.best_path)
    {
        progress.best_path = result.path;
    }

    for (typename std::set<search_node<S, P> >::iterator i =
             progress.INCONS.begin();
         i != progress.INCONS.end(); i++)
    {
        result.inconsistent.insert(i->state);
    }

    std::priority_queue<search_node<S, P> > cop(progress.OPEN);
    while(!cop.empty())
    {
        result.inconsistent.insert(cop.top().state);
        cop.pop();
    }
    request.add_solution(result);
    return progress.goal_found;
}

template <class S, typename P>
float min_gh(search_progress_info<S, P>& progress)
{
    std::priority_queue<search_node<S, P> > OPEN_copy(progress.OPEN);

    // Min over s in OPEN, INCONS g(s) + h(s)
    float min_g_plus_h = (progress.costs[OPEN_copy.top().state] +
                          heuristic(OPEN_copy.top().state));
    while (!OPEN_copy.empty())
    {
        OPEN_copy.pop();
        float g_h = (progress.costs[OPEN_copy.top().state] +
                     heuristic(OPEN_copy.top().state));
        if ( g_h < min_g_plus_h)
        {
            min_g_plus_h = g_h;
        }
    }
    for (typename std::set<search_node<S, P> >::iterator i =
             progress.INCONS.begin();
         i != progress.INCONS.end(); i++)
    {
        float g_h = (progress.costs[i->state] + heuristic(i->state));
        if ( g_h < min_g_plus_h)
        {
            min_g_plus_h = g_h;
        }
    }
    return min_g_plus_h;
}

template <typename S, typename P>
void arastar(search_request<S, P>& request)
{
    std::clock_t t;
    t = std::clock();
    request.go();

    search_progress_info<S, P> progress;
    progress.OPEN = std::priority_queue<search_node<S, P> >();
    progress.best_path = std::vector<P>();
    progress.INCONS = std::set<search_node<S, P> >();
    progress.costs = std::map<S, float>();
    progress.goal_found = false;

    // g(goal) = inf; g(start) = 0;
    progress.costs[request.begin()] = 0;

    search_node<S, P> snode;
    snode.state = request.begin();
    snode.f_value = (progress.costs[snode.state] +
                     request.check_epsilon()*heuristic(snode.state));
    progress.OPEN.push(snode);

    bool killed = false;
    progress.goal_found = improve_path<S, P>(request, progress);
    if (progress.OPEN.empty()) return;
    //best_path = solutions->at(0).path;

    // g(goal)/min{s E OPEN U INCONS} (g+h)
    float alt = progress.costs[progress.goal] / min_gh<S, P>(progress);

    t = std::clock() - t;

    float e_prime = request.check_epsilon();
    if (alt < e_prime) e_prime = alt;
    std::cout << "The first solution is suboptimal by: " <<
        e_prime << ", ";
    std::cout << request.copy_solutions().at(0).expanded.size()
              << " expansions, ";
    std::cout << ((float)t) / CLOCKS_PER_SEC << " s"  << std::endl;
    if (request.check_killed()) return;

#ifdef FIRST_SOL
    return;
#endif

    while (e_prime > 1.f)
    {
        while (request.check_paused())
        {
            if (request.check_pause_waiting())
            {
                request.set_pause_waiting(false);
            }
            sleep(0.1);
        }

        // decrease epsilon
        float tmp = request.check_epsilon();
#ifdef MAZE_FIGURE
        request.set_epsilon(tmp - 0.5f);
#else
        request.set_epsilon(tmp - 1.f);
#endif

        // Move states from INCONS to OPEN
        for (typename std::set<search_node<S, P> >::iterator i =
                 progress.INCONS.begin();
             i != progress.INCONS.end(); i++)
        {
            progress.OPEN.push(*i);
        }
        progress.INCONS.clear();

        // Update priorities of all states in OPEN
        std::vector<search_node<S, P> > OPEN_update;
        while (!progress.OPEN.empty())
        {
            OPEN_update.push_back(progress.OPEN.top());
            progress.OPEN.pop();
        }
        for (typename std::vector<search_node<S, P> >::iterator o =
                 OPEN_update.begin();
             o != OPEN_update.end(); o++)
        {
            o->f_value = (progress.costs[o->state] +
                          request.check_epsilon()*heuristic(o->state));
            progress.OPEN.push(*o);
        }

        progress.goal_found = improve_path<S,P>(request, progress);
        if (request.check_killed()) return;
        if (progress.OPEN.empty()) return;

        // g(goal)/min{s E OPEN U INCONS} (g+h)
        float alt = progress.costs[progress.goal] / min_gh<S,P>(progress);

        e_prime = request.check_epsilon();
        if (alt < e_prime) e_prime = alt;
        std::cout << "Epsilon is: " << request.check_epsilon() <<
            ", Solution is suboptimal by: " << e_prime << ", ";
        std::cout << request.copy_solutions().at(request.num_solutions()-1).expanded.size()
                  << " expansions" << std::endl;

    }
}

template <class S, typename P>
search_result<S, P> astar(search_request<S, P>& req)
{
    req.kill();
    arastar<S, P>(req);
    return req.copy_solutions().at(0);
}
