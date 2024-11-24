#ifndef __TASK_TYPE_HPP__
#define __TASK_TYPE_HPP__

#define TASK_A_PROB 0.33
// #define TASK_B_PROB 0.66

#define RAND ((float) rand()/RAND_MAX)

#define strType(x) ((char) ((int)x + 65))

typedef enum {
    A,
    B,
    UNKNOWN
} TaskType;

extern const TaskType typeMap[];
extern const double taskTime[2][2];

TaskType getType(int value);
TaskType generate_random_task();
double get_task_time(TaskType robot_type, TaskType event_type);
double* getColor(TaskType t);

#endif // __TASK_TYPE_HPP__