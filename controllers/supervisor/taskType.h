#define strType(x) ((char) ((int)x + 65))

typedef enum {
    A,
    B,
    UNKNOWN
} TaskType;

const TaskType typeMap[] = {
    A, // 0
    A, // 1
    B, // 2
    B, // 3
    B  // 4
};

TaskType getType(int value) {
    if (value >= 0 && value < sizeof(typeMap) / sizeof(typeMap[0])) {
        return typeMap[value];
    } else {
        return UNKNOWN; // Return UNKNOWN for any other value
    }
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