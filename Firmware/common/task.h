#ifndef TASK_H
#define TASK_H


#include <stdint.h>

// ********* TASKS ************************************** //

#define CHECK_PERIOD(TASK, LOOPTICK) 	((LOOPTICK - TASK.lastTick >= TASK.period) && (TASK.enable))

struct FixedTimeTask{
    uint32_t lastTick;
    uint32_t period;	 // ms
	bool enable;
}FixedTimeTask;

static struct FixedTimeTask createTask(uint32_t period){
	struct FixedTimeTask task;
	task.lastTick = 0;
	task.period = period;
	task.enable = true;
	return task;
}

static inline void disableTask(struct FixedTimeTask *t){t->enable = false;}
static inline void enableTask(struct FixedTimeTask *t){t->enable = true;}

#endif // TASK_H