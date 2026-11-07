# Controller Story

## Handling Inputs
I want to be able to have both "reactionary" or "sample-based" modes of communication between the input and the controller.

### Reactionary
Upon arrival of the data the controller executes.
* This probably wont be a thing for us to do (NOTODO).

### Sample-based
Fixed rate per-each controller.

## Input Timing Requirements
We need to have a core piece of the controller be that the inputs must be coming in at a reasonable time period.
* As a part of the config for each controller we need to have a maximum input period that can ellapse before controller disables output.

## Drivetrain Requirements
* We want to be able to have controllers that require the drivetrain to be in a certain state / control mode.
* We want to be able to output different types of output from each controller.
* The controller should have access to the parameter server and be "Configurable".

# Controller Manager Story

Based on: https://github.com/hytech-racing/MCU/blob/adc599c9a2a3d1afe4ee22fcad0fd1116c474500/lib/systems/include/TorqueControllerMux.h

TODO: Look into sharing tc mux between MCU and drivebrain software

## Requirements
- Shall handle ability to be constructed to contain the controllers, handle providing them the inputs from state estimation and / or drivers.
- Controller manager shall not affect the output of any controller.
- Each controller shall be able to handle re-configuration at runtime, even while being the "active" controller, however the controller manager shall handle when its configuration gets updated.
- Vehicle manager shall handle evaluating of each controller at their desired rates and be able to switch between.
- Vehicle manager shall be able to change the mode of the drivetrain dependent on what mode the desired controller wants the drivetrain to be in (speed / torque).

## TODOs
- [ ] implement handling for switching between controllers -> yoink logic from new tcmux
- [ ] implement function to pass in vehicle state to the active controller

## General Goals
What I want to be able to do with the vehicle manager is be general about what controllers we are swapping between and evaluating during runtime. Each controller (for now) will be stack allocated, however they wont always be running at the same time. We also want the ability to pause evaluation of all controllers, but this isnt something that we have to have at the start.

We also want to be able to expose an interface to the user to use GRPC to interact with this vehicle manager to allow for control pieces of our car, like say run specific routines like start, stop lap time recording or set start location of lap timing. This, however, probably wouldnt be a part of the vehicle manager itself.

## Implementation Details
For the sake of the controllers, I want to have a direct connection between the state estimator and the Controller Manager. The controller manager will sample the state estimator (there will be a single state that the state estimator updates internally).

*Note:* We need to make sure that we copy over the vehicle state here after locking onto the mutex and then unlock.

*Question:* How are we going to prevent mutex deadlocking on the shared state?
*Answer:* We will do this by having the state estimator and the controller ticking in the same loop so no need for worrying about threading.
