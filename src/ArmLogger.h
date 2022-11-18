#ifndef ArmLogger_h
#define ArmLogger_h

// class ArmLogger{
    extern unsigned long logTime;
    enum logType {DATA, LOG};

    // public:
    void outputLog(logType l, char* message);
    void printMessage(String message);
    void printMessageFloat(String message, float value);
    void printMessageHex(String message, int value);
    void printMessageInt(String message, int value);
    void angleLog(float angle);
// };
#endif