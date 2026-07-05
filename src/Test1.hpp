#ifndef TEST1_H
#define TEST1_H

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>

class Controller;

class RobotSystem;

class Test1 {
public:
    Test1(RobotSystem *);

    virtual ~Test1();

    virtual void testInitialization() = 0;

    void getCommand(void *_command);
    int getPhase() { return phase_; }

protected:
    virtual int nextPhase(const int &phase) = 0;

    bool b_first_visit_;
    int phase_;
    std::vector<Controller *> state_list_;
};
#endif
