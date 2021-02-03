#include <psaf_local_planner/plugin_local_planner.h>


namespace psaf_local_planner
{
    LocalPlannerStateMachine::LocalPlannerStateMachine(){
        this->state = LocalPlannerState::UNKNOWN;
    }
    void LocalPlannerStateMachine::init(){
        this->state = LocalPlannerState::DRIVING;
    }

    LocalPlannerState LocalPlannerStateMachine::getState(){
        return this->state;
    }

    void LocalPlannerStateMachine::setState(LocalPlannerState state){
        this->state = state;
    }

}