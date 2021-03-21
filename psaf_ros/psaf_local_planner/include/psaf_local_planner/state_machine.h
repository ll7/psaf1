#include <psaf_messages/TrafficLight.h>
#ifndef PSAF_STATE_MACHINE_H
#define PSAF_STATE_MACHINE_H

namespace psaf_local_planner {

    enum class LocalPlannerState {

        UNKNOWN,
        /**
         * Car is driving on road
         */
        DRIVING,
        /**
         * There is upcoming traffic light
         */
        TRAFFIC_LIGHT_NEAR,
        /**
         * Traffic light is green and we can pass the intersection
         */
        TRAFFIC_LIGHT_GO,
        /**
         * Approaching to the intersection but have to stop
         */
        TRAFFIC_LIGHT_WILL_STOP,
        /**
         * The traffic light is red, so reduce the speed and hope it will turn green
         */
        TRAFFIC_LIGHT_SLOW_DOWN,
        /**
         * Waiting at the traffic light
         */
        TRAFFIC_LIGHT_WAITING,
        /**
         * The stop sign/mark is close
         */
        STOP_NEAR,
        /**
         * The stop sign/mark is visible ->  stop at the stop line
         */
        STOP_WILL_STOP,
        /**
         * Waiting at the stop line until the intersection is clear
         */
        STOP_WAITING,
        /**
         * Intersection is clear. Let's go
         */
        STOP_GO
    };

    /**
     * The local planner state machine when driving with traffic rules
     */
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
        virtual bool isInTrafficLightStates();

        /**
          * Returns whether the state machine is in the stop sign or stop mark handling region
          * (=all states that are related to the correct behaviour near a stop sign or stop mark)
          * @return whether the state machine is in the stop sign or stop mark handling region
          */
        virtual bool isInStopStates();

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
        virtual void updateState(bool trafficLightDetected, bool stopDetected,
                                 psaf_messages::TrafficLight trafficLightKnowledge, double stoppingDistance,
                                 double currentSpeed, double distanceToStopLine, bool isIntersectionClear,
                                 double currentTimeSec);

        /**
         * Returns the current state as text for e.g. logging
         * @return the current state as string
         */
        std::string getTextRepresentation();

    protected:
        LocalPlannerState state;
        /**
          * Ros time in sec when we entered the state STOP_GO
          * infinity represents that no value was set
          */
        double start_time_stop_go;

    private:
        /**
         * Ros time in sec when we entered the state STOP_WAITING
         * 0 represents that no value was set
         */
        double start_time_stop_waiting;

        /**
        * Ros time in sec when the car was waiting at a traffic light and had no knowled about the traffic light state
        * infinity represents that no value was set
        */
        double start_time_waiting_without_tl_state;
    };

    /**
     * State machine when driving without traffic rules
     * -> Handles the traffic lights as stop signs
     */
    class LocalPlannerStateMachineWithoutTrafficRules: public LocalPlannerStateMachine{
    public:
        /**
        * Constructor: Nothing happens here, other than standard values are getting inited
        */
        LocalPlannerStateMachineWithoutTrafficRules();

        ~LocalPlannerStateMachineWithoutTrafficRules();

        void updateState(bool trafficLightDetected, bool stopDetected,
                         psaf_messages::TrafficLight trafficLightKnowledge, double stoppingDistance,
                         double currentSpeed, double distanceToStopLine, bool isIntersectionClear,
                         double curTimeSec) override;

        bool isInTrafficLightStates() override;
    };
} // namespace psaf_local_planner

#endif