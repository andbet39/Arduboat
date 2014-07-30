#include <FastSerial.h>
#include <stdint.h>
#import <Arduino.h>

#ifndef AS_Scheduler_h
#define AS_Scheduler_h




class AS_Scheduler {
  
public:
    typedef void (*task_fn_t)(void);
    
	struct Task {
		task_fn_t function;
		uint16_t interval_ticks;
		uint16_t max_time_micros;
	};


    // initialise scheduler
	void init(const Task *tasks, uint8_t num_tasks);
    
	// call when one tick has passed
	void tick(void);
    
	// run the tasks. Call this once per 'tick'.
	// time_available is the amount of time available to run
	// tasks in microseconds
	void run(uint16_t time_available);
    
    // current running task, or -1 if none. Used to debug stuck tasks
    static int8_t current_task;
    
    float load_average(uint32_t tick_time_usec) const;

    // return the number of microseconds available for the current task
	uint16_t time_available_usec(void);


private:
    // progmem list of tasks to run
	const struct Task *_tasks;
	
	// number of tasks in _tasks list
	uint8_t _num_tasks;
    
	// number of 'ticks' that have passed (number of times that
	// tick() has been called
	uint16_t _tick_counter;
    
	// tick counter at the time we last ran each task
	uint16_t *_last_run;
    
	// number of microseconds allowed for the current task
	uint32_t _task_time_allowed;
    
	// the time in microseconds when the task started
	uint32_t _task_time_started;
    
    // number of spare microseconds accumulated
    uint32_t _spare_micros;
    
    // number of ticks that _spare_micros is counted over
    uint8_t _spare_ticks;


};

#endif
