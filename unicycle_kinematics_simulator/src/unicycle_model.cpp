#include "unicycle_model.h"

unicycle_model::unicycle_model(double deltaT) : dt(deltaT), t(0.0), model_state(NUM_CONFIG)
{
    // configuration variables
    for(int i = 0; i < NUM_CONFIG; i++) {
        model_config.push_back(0.0);
    }

    // state variables
    for(int i = 0; i < NUM_STATE; i++) {
        model_state.push_back(0.0);
    }

    // input variables
    for(int i = 0; i < NUM_INPUT; i++) {
        model_u.push_back(0.0);
    }
}

void unicycle_model::setConfigValues(state_t &config)
{
    // initial state values
    for (int i = 0; i < NUM_CONFIG; i++) {
        model_config[i] = config[i];
    }
}

void unicycle_model::setStateValues(state_t &state)
{
    // initial state values
    for (int i = 0; i < NUM_STATE; i++) {
        model_state[i] = state[i];
    }
}

void unicycle_model::setInputValues(input_t &u)
{
    // setting new input values
    for (int i = 0; i < NUM_INPUT; i++) {
        model_u[i] = u[i];
    }
}

void unicycle_model::updateModel()
{
    // kinematic model equations update
    model_state[D_X] = model_u[VEL_LIN] * std::cos(model_state[THETA]);
    model_state[D_Y] = model_u[VEL_LIN] * std::sin(model_state[THETA]);
    model_state[D_THETA] = model_u[VEL_ANG];

    // configuration variables update
    model_config[X] += model_state[D_X] * dt;
    model_config[Y] += model_state[D_Y] * dt;
    model_config[THETA] += model_state[D_THETA] * dt;

    // degree bound [0, 360]
    if (model_config[THETA] > 360) {
        model_config[THETA] -= 360;
    }

    // update time
    t += dt;
}