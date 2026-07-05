#include "Test1.hpp"
#include "Controller.hpp"
#include <Utils/DataManager.hpp>

Test1::Test1(RobotSystem *robot) : b_first_visit_(true) {
    DataManager::GetDataManager()->RegisterData(&phase_, INT, "phase");
}

Test1::~Test1() {
}

void Test1::getCommand(void *command) {
    if (b_first_visit_) {
        state_list_[phase_]->FirstVisit();
        b_first_visit_ = false;
    }

    state_list_[phase_]->OneStep(command);

    if (state_list_[phase_]->EndOfPhase()) {
        state_list_[phase_]->LastVisit();
        phase_ = nextPhase(phase_);
        b_first_visit_ = true;
    }
}
