#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_
#include <mutex>

class StateMachine
{
public:
    StateMachine()
    {
        state = State_SystemIdle;
    }

    enum State
    {
        State_SystemIdle = 0,

        State_SuspendTracking = 1,
        State_VertexTracking = 2,
        State_CurveTracking = 3,
        State_CompleteTracking = 4,

        State_CurvePathRecording = 10,
        State_VertexPathRecording = 11,
    };

    bool isRecording()
    {
        if(state == State_CurvePathRecording ||
            state == State_VertexPathRecording)
            return true;
        return false;
    }

    bool isTracking()
    {
        if(state >= State_SuspendTracking && 
           state <= State_CompleteTracking)
            return true;
        return false;
    }

    bool isIdle()
    {
        return state == State_SystemIdle;
    }

    void set(uint8_t _state)
    {
        mutex.lock();
        state = _state;
        mutex.unlock();
    }

    uint8_t get() const
    {
        return state;
    }
    
private:
    uint8_t state;
    std::mutex mutex;
};

#endif