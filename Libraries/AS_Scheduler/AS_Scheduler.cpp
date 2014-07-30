


#include <AS_Scheduler.h>

void AS_Scheduler::init(const Task *tasks, uint8_t num_tasks){

    _tasks = tasks;
    _num_tasks = num_tasks;
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _tick_counter = 0;


}

// one tick has passed
void AS_Scheduler::tick(void)
{
    _tick_counter++;
}


/*
 run one tick
 this will run as many scheduler tasks as we can in the specified time
 */
void AS_Scheduler::run(uint16_t time_available)
{
    uint32_t run_started_usec = micros();
    uint32_t now = run_started_usec;
    
    for (uint8_t i=0; i<_num_tasks; i++) {
        uint16_t dt = _tick_counter - _last_run[i];
        uint16_t interval_ticks = _tasks[i].interval_ticks;
        if (dt >= interval_ticks) {
            // this task is due to run. Do we have enough time to run it?
            _task_time_allowed = _tasks[i].max_time_micros;
            
            if (dt >= interval_ticks*2) {
                // we've slipped a whole run of this task!
            }
            
            if (_task_time_allowed <= time_available) {
                // run it
                _task_time_started = now;
                task_fn_t func = (task_fn_t)_tasks[i].function;
                //current_task = i;
                func();
                //current_task = -1;
                
                // record the tick counter when we ran. This drives
                // when we next run the event
                _last_run[i] = _tick_counter;
                
                // work out how long the event actually took
                now = micros();
                uint32_t time_taken = now - _task_time_started;
                
                if (time_taken > _task_time_allowed) {

                }
                if (time_taken >= time_available) {
                    goto update_spare_ticks;
                }
                time_available -= time_taken;
            }
        }
    }
    
    // update number of spare microseconds
    _spare_micros += time_available;
    
update_spare_ticks:
    _spare_ticks++;
    if (_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }
}

/*
 return number of micros until the current task reaches its deadline
 */
uint16_t AS_Scheduler::time_available_usec(void)
{
    uint32_t dt = micros() - _task_time_started;
    if (dt > _task_time_allowed) {
        return 0;
    }
    return _task_time_allowed - dt;
}
