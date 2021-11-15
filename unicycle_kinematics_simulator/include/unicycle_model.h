#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

/* @constant */
#define NUM_CONFIG  3 // number of configuration variables
#define NUM_STATE   3 // number of state variables
#define NUM_INPUT   4 // number of input command variables

/* @type : state variable data type */
typedef std::vector<double> config_t;
/* @enum : in order to distinguish vector elements*/
enum { X, Y, THETA };

/* @type : configuration variable data type */
typedef std::vector<double> state_t;
/* @enum : in order to distinguish vector elements*/
enum { D_X, D_Y, D_THETA };

/* @type : input command */
typedef std::vector<double> input_t;
/* @enum : in order to distinguish vector elements */
enum { VEL_LIN, VEL_ANG };

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
};