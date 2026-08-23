# 3D Interceptor UAV – PPN Interception Simulation
A MATLAB-based 3D interceptor UAV simulation using Proportional Navigation (PPN) to pursue a maneuvering target. The project visualizes UAV/target trajectories and analyzes range, closing velocity, LOS rate, and guidance acceleration during interception.

## 1. Simulation Overview

The simulation models a **3D interceptor UAV pursuing and intercepting a maneuvering target aircraft** using **Proportional Navigation (PPN)** guidance. The UAV initially flies directly toward the target and then smoothly transitions to PPN pursuit before attempting interception.

## 2. Simulation Parameters
These parameters are defined directly in the simulation code.

| Parameter               |             Value | Purpose                                                                                                          |
| ----------------------- | ----------------: | ---------------------------------------------------------------------------------------------------------------- |
| Simulation time step    |           0.002 s | Provides a small integration step for accurate and smooth 3D motion and guidance calculations.                   |
| Maximum simulation time |              40 s | Defines the maximum duration of the simulation if interception does not occur earlier.                           |
| Navigation constant, N  |                 4 | Determines the strength of the PPN guidance response to LOS motion and closing velocity.                         |
| Intercept radius        |               5 m | Defines the maximum separation at which the UAV is considered to have intercepted the target.                    |
| Initial UAV speed       |           280 m/s | Sets the UAV's starting and controlled flight speed during the interception simulation.                          |
| Target initial position | [2000, 0, 1000] m | Defines the target's starting location in the 3D simulation environment.                                         |
| Target velocity         |  [-100, 0, 0] m/s | Defines the target's initial velocity and direction of motion.                                                   |
| Direct-to-target phase  |           0–1.5 s | Allows the UAV to initially align toward the target before introducing PPN guidance.                             |
| PPN transition          |         1.5–3.5 s | Gradually blends direct guidance with PPN to avoid an abrupt change in acceleration commands.                    |
| PPN pursuit             |       After 3.5 s | Enables pure PPN guidance for continuous interception of the maneuvering target.                                 |
| Transition acceleration |          120 m/s² | Limits the steering acceleration during the initial guidance and transition phases for smoother UAV maneuvering. |
| UAV speed control       |    Constant speed | Maintains a fixed UAV speed so that the simulation primarily evaluates the guidance and trajectory response.     |


## 3. Target Maneuver
Unlike a simple straight-line target, the simulated target performs a **smooth zigzag/S-turn maneuver** in heading, combined with a slower altitude variation. Small positional jitter is also added to represent disturbances or turbulence. The target maintains a constant speed while its direction changes.

## 4. Guidance Method

The main guidance algorithm is **Pure Proportional Navigation (PPN)**. The simulation calculates:

* Relative position and range
* Relative velocity
* Closing velocity
* Line-of-sight (LOS) angular velocity
* Required PPN acceleration

The PPN acceleration is calculated using the navigation constant, closing velocity, LOS rate, and UAV velocity direction.

## 5. Flight Phases

### Phase 1 – Direct-to-Target

The UAV initially points toward the target and uses steering acceleration to reduce the direction error. This phase lasts **1.5 seconds**.

### Phase 2 – PPN Transition

From **1.5 to 3.5 seconds**, direct guidance is gradually blended with PPN guidance. This provides a smooth transition rather than an abrupt change in the control command.

### Phase 3 – PPN Pursuit

After **3.5 seconds**, the UAV uses pure PPN guidance to continuously correct its trajectory toward the maneuvering target.

## 6. Simulation Outputs

The simulation provides a 3D visualization of both aircraft and their trajectories. Four telemetry plots are also generated:

1. **Range R(t)** – distance between UAV and target. The range is the Euclidean distance between the target position and UAV position:
$$
\mathbf{r} = \mathbf{p}_T - \mathbf{p}_U
$$

$$
R(t) = \|\mathbf{r}\| = \sqrt{(x_T - x_U)^2 + (y_T - y_U)^2 + (z_T - z_U)^2}
$$

   
3. **Closing Velocity** – rate at which the UAV and target approach each other.
4. **LOS Angular Rate** – rate of change of the line-of-sight direction.
5. **Commanded Acceleration** – acceleration required by the guidance system.

The animation displays the UAV and target positions, trajectories, current flight phase, range, navigation constant, and simulation time.

## 7. Interception Condition

An interception is declared when the separation between the UAV and target becomes less than the specified **5 m intercept radius**. At interception, the simulation records the flight phase, interception time, final range, closing velocity, LOS angular rate, acceleration, and UAV speed.

## 8. Conclusion

The simulation demonstrates a **3D UAV interception scenario using PPN guidance against a maneuvering target**. The combination of direct-to-target guidance, smooth PPN transition, and pure PPN pursuit provides a complete guidance sequence. The graphical telemetry allows the effectiveness of the guidance algorithm to be evaluated through range, closing velocity, LOS rate, and acceleration behavior.

