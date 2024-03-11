# motor-canopen: cia402 motor control
Implements various parts of a cia402 enabled motor.

To use this follow a few steps:
#### Create your State:
Create a new static `State` class implementing the interface defined in `state_base.hpp`.
For each variable you want to be available as a Canopen Register, implement a handler in `registerHandlers`.
Each update cycle the `update` method will be called, here you should update any variables that will be used by other protocols, as this is called before they are updated.

#### Select your protocols

Select the appropriate Protocols you want your motor to support, such as `CurrentProtocol` and `VelocityProtocol`.
To use some protocols you will need to have certain variables in `State`, for example the `IdentityProtocol` will need an instance of the `Identity` struct in your `State`, called `identity_`. 
Look into `motor_state.hpp` to see an example implementation.

<TODO:> Define what is needed in the Protocol class, so noone needs to read through the code.

#### Create your motor control
Create a class for your motor using your desired Protocols:
```
template<size_t id>
using MotorControl_t = MotorControl<id, State,CurrentProtocol<id>,...>;
using MotorControl0 = MotorControl_t<0>;
```
The id parameter allows multiple instances of canopen to be run concurrently on the same device.

#### Using the control
Initialize the canopen instance using `MotorControl0::initialize` with the canopen node ID.
Call the `update` and `processMessage` functions in a loop to handle all variable updates etc. and simple read the updated state variables from your `State`.


### Position/Velocity/Current Protocol control flow
This is the main part of the BLDC controller implementation.
It is split into Protocol and Control parts. This allows an implementation using the `PositionProtocol` to not have to support the `VelocityProtocol` and `CurrentProtocol`. Each of these three Protocols will update the variables `outputPWM_` and `outputCurrentLimit_` in your `State`.

When using the `PositionProtocol` you will automatically use the `VelocityControl` and `CurrentControl`.
The `PositionProtocol` and `VelocityControl` both implement a PID controller, `CurrentControl` simply generates the appropriate PWM and current to configure your motor with. 