#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

/* @constant */
#define NUM_CONFIG  3 // number of configuration variables
#define NUM_STATE   5 // number of state variables
#define NUM_INPUT   4 // number of input command variables
#define NUM_PARAM   2 // number of robot parameters for dynamics

/* @type : state variable data type */
typedef std::vector<double> config_t;
/* @enum : in order to distinguish vector elements*/
enum { X, Y, THETA };

/* @type : configuration variable data type */
typedef std::vector<double> state_t;
/* @enum : in order to distinguish vector elements*/
enum { D_X, D_Y, D_THETA, D_VEL_LIN, D_VEL_ANG };

/* @type : input command */
typedef std::vector<double> input_t;
/* @enum : in order to distinguish vector elements */
enum { VEL_LIN, VEL_ANG, TORQ_TAU_1, TORQ_TAU_2 };

/* @type : model params */
typedef std::vector<double> param_t;
/* @enum : in order to distinguish vector elements */
enum { MASS, INERTIA };

/* @class : unicycle model */
class unicycle_model
{
    public:
        // constructor
        unicycle_model(double deltaT);

        // setters
        void setConfigValues(config_t &config);
        void setStateValues(state_t &state);
        void setInputValues(input_t &u);
        void setParamValues(param_t &param);

        // getters
        void getConfigValues(config_t &config) {
            // returning the current state through methods addresses params
            for(int i = 0; i < NUM_CONFIG; i++) {
                config[i] = model_config[i];
            }
        };

        void getStateValues(state_t &state) {
            // returning the current state through methods addresses params
            for(int i = 0; i < NUM_STATE; i++) {
                state[i] = model_state[i];
            }
        };

        void getInput(input_t &u) {
            // returning the current state through methods addresses params
            for(int i = 0; i < NUM_INPUT; i++) {
                u[i] = model_u[i];
            }
        };

        void getParam(param_t &param) {
            // returning the current state through methods addresses params
            for(int i = 0; i < NUM_PARAM; i++) {
                param[i] = model_param[i];
            }
        };

        void getTime(double &time) {
            time = t;
        };

        // the same of "integrate" in ode
        void updateModel();
    
    private:
        // Simulator and integrator variables
        double t, dt;

        // configuration variables
        config_t model_config;

        // state variables
        state_t model_state;
        
        // input variables
        input_t model_u;

        // model params
        param_t model_param;
};