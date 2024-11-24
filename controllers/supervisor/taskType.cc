#include "include/taskType.hpp"
#include <stdlib.h>

const TaskType typeMap[] = {
    A, // 0
    A, // 1
    B, // 2
    B, // 3
    B  // 4
};

const double taskTime[2][2] = {
    {3, 5}, {9, 1}
};

TaskType getType(int value) {
    if (value >= 0 && value < sizeof(typeMap) / sizeof(typeMap[0])) {
        return typeMap[value];
    } else {
        return UNKNOWN; // Return UNKNOWN for any other value
    }
}

TaskType generate_random_task() {
    if(RAND <= TASK_A_PROB)
        return A;
    else  
        return B;
}

double get_task_time(TaskType robot_type, TaskType event_type) {
    if(robot_type == UNKNOWN || event_type == UNKNOWN) return -1;
    return taskTime[int(robot_type)][int(event_type)];
}

double* getColor(TaskType t) {
    static double arr[3]; 
    switch (t) {
        case A:
            arr[0] = 1.0;
            arr[1] = 0.0;
            arr[2] = 0.0;
            break;

        case B:
            arr[0] = 0.0;
            arr[1] = 0.0;
            arr[2] = 1.0;
            break;
        
        default:
            arr[0] = 0.0;
            arr[1] = 0.0;
            arr[2] = 0.0;
            break;
    }

    return arr;
}