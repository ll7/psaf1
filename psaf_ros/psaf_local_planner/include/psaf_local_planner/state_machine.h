#include <psaf_messages/TrafficLight.h>
namespace psaf_local_planner
{

    enum class LocalPlannerState
    {
        
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
        DONE
    };

    class LocalPlannerStateMachine
    {
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

        //TODO: remove
        /**
         * Deprecated remove in future
         */
        void setState(LocalPlannerState state);

        void updateState(bool trafficLightDetected,psaf_messages::TrafficLight trafficLightKnowledge,double stoppingDistance,
            double current_speed, double distanceToStopLine);
    private:
        LocalPlannerState state;
    };
} // namespace psaf_local_planner