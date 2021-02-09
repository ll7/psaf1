#include <psaf_messages/TrafficLight.h>

namespace psaf_local_planner {

    enum class LocalPlannerState {

        UNKNOWN,
        /**
         * Car is driving on road
         */
        DRIVING,
        TRAFFIC_LIGHT_NEAR,
        TRAFFIC_LIGHT_GO,
        TRAFFIC_LIGHT_WILL_STOP,
        TRAFFIC_LIGHT_SLOW_DOWN,
        TRAFFIC_LIGHT_WAITING,
        STOP_NEAR,
        STOP_WILL_STOP,
        STOP_WAITING,
        STOP_GO
    };

    class LocalPlannerStateMachine {
    public:
        /**
         * Constructor: Nothing happens here, other than standard values are getting inited
         */
        LocalPlannerStateMachine();

        ~LocalPlannerStateMachine();

        /**
         * Initialize the state machine 
         */
        void init();

        /**
         * Returns the current state
         * @return the current state
         */
        LocalPlannerState getState();


        /**
         * Returns whether the state machine is in the traffic light handling region
         * (=all states that are related to the correct behaviour near a traffic light)
         * @return whether the state machine is in the traffic light handling region
         */
        bool isInTrafficLightStates();

        /**
          * Returns whether the state machine is in the stop sign or stop mark handling region
          * (=all states that are related to the correct behaviour near a stop sign or stop mark)
          * @return whether the state machine is in the stop sign or stop mark handling region
          */
        bool isInStopStates();

        /**
         * Reset the state machine when a new plan is received
         */
        void reset();

        /**
         * Updates the state regarding the given parameters
         * @param trafficLightDetected
         * @param trafficLightKnowledge
         * @param stoppingDistance
         * @param currentSpeed
         * @param distanceToStopLine
         */
        void updateState(bool trafficLightDetected, bool stopDetected,
                         psaf_messages::TrafficLight trafficLightKnowledge, double stoppingDistance,
                         double currentSpeed, double distanceToStopLine, bool isIntersectionClear);

        /**
         * Returns the current state as text for e.g. logging
         * @return the current state as string
         */
        std::string getTextRepresentation(){
            switch (this->state) {
                case LocalPlannerState::DRIVING:
                    return "Driving";
                case LocalPlannerState::TRAFFIC_LIGHT_NEAR:
                    return "Traffic light: near";
                case LocalPlannerState::TRAFFIC_LIGHT_GO:
                    return "Traffic light: go";
                case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
                    return "Traffic light: will stop";
                case LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN:
                    return "Traffic light: slow down";
                case LocalPlannerState::TRAFFIC_LIGHT_WAITING:
                    return "Traffic light: waiting";
                case LocalPlannerState::STOP_NEAR:
                    return "Stop: near";
                case LocalPlannerState::STOP_WILL_STOP:
                    return "Stop: will stop";
                case LocalPlannerState::STOP_WAITING:
                    return "Stop: waiting";
                case LocalPlannerState::STOP_GO:
                    return "Stop: go";
            }
            return "unknown state";
        }

    private:
        LocalPlannerState state;
    };
} // namespace psaf_local_planner