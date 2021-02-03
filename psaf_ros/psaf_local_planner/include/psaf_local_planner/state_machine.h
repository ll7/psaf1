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

        void init();

        LocalPlannerState getState();
        void setState(LocalPlannerState state);
    private:
        LocalPlannerState state;
    };
} // namespace psaf_local_planner