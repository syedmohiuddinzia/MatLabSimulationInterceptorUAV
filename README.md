# 3D Interceptor UAV – PPN Interception Simulation
A MATLAB-based 3D interceptor UAV simulation using Proportional Navigation (PPN) to pursue a maneuvering target. The project visualizes UAV/target trajectories and analyzes range, closing velocity, LOS rate, and guidance acceleration during interception.

## 1. Simulation Overview

The simulation models a **3D interceptor UAV pursuing and intercepting a maneuvering target aircraft** using **Proportional Navigation (PPN)** guidance. The UAV initially flies directly toward the target and then smoothly transitions to PPN pursuit before attempting interception.

## 2. Simulation Parameters

| Parameter               |             Value |
| ----------------------- | ----------------: |
| Simulation time step    |           0.002 s |
| Maximum simulation time |              40 s |
| Navigation constant, N  |                 4 |
| Intercept radius        |               5 m |
| Initial UAV speed       |           280 m/s |
| Target initial position | [2000, 0, 1000] m |
| Target velocity         |  [-100, 0, 0] m/s |
| Direct-to-target phase  |           0–1.5 s |
| PPN transition          |         1.5–3.5 s |
| PPN pursuit             |       After 3.5 s |
| Transition acceleration |          120 m/s² |
| UAV speed control       |    Constant speed |

These parameters are defined directly in the simulation code.

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

1. **Range R(t)** – distance between UAV and target.
2. **Closing Velocity** – rate at which the UAV and target approach each other.
3. **LOS Angular Rate** – rate of change of the line-of-sight direction.
4. **Commanded Acceleration** – acceleration required by the guidance system.

The animation displays the UAV and target positions, trajectories, current flight phase, range, navigation constant, and simulation time.

## 7. Interception Condition

An interception is declared when the separation between the UAV and target becomes less than the specified **5 m intercept radius**. At interception, the simulation records the flight phase, interception time, final range, closing velocity, LOS angular rate, acceleration, and UAV speed.

## 8. Conclusion

The simulation demonstrates a **3D UAV interception scenario using PPN guidance against a maneuvering target**. The combination of direct-to-target guidance, smooth PPN transition, and pure PPN pursuit provides a complete guidance sequence. The graphical telemetry allows the effectiveness of the guidance algorithm to be evaluated through range, closing velocity, LOS rate, and acceleration behavior.

