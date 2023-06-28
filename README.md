# orbit-sim

A simple ROS-based 2D orbital mechanics simulator 

### Key features
 - numerical simulation of spacecraft 2d motion around fixed central body
 - rviz visualization and display of states and orbital elements
 - user-defined initial conditions and kick-burn thrust
 - execution of hohman transfers

### Main requisites
- ROS Melodic
- Moba XTerm
- Ubuntu 18.4

### Short background

The differential equation describing the spacecraft motion in a radial frame is derived from Newton's gravitational force  
```math
\ddot{r} = -\frac{\mu}{r^{2}}
```

A kick-burn is modeled as an instantaneous thrust given by a delta velocity at a time instant
```math
v(t_{+}) = v(t_{-}) + \Delta{v(t)}
```

A single spacecraft (SC) state vector consists of its cartesian position and velocity - which are in fact numerically integrated. It is also of substancial importance to know the orbital elements, what is possible through a state transformation.
```math
\begin{bmatrix}  x & y & v_x & v_y \end{bmatrix} \rightarrow \begin{bmatrix}  a & e & \omega & \theta \end{bmatrix}
```
where $a$ is the semi-major axis,  $e$ is the eccentricity, $\omega$ is the argument of the periapsis, and $\theta$ the true anomaly. Four elements are sufficient to describe a 2d orbital motion, while the first three describe the orbit itself.

In this implementation, a global reference frame is fixed in the central body, while a NTW in-track frame is fixed in the moving small body. The initial states must be given wrt. the global coordinate system, but the thrust action must be determined in the local base.

<p align="center">
  <img src="https://github.com/lucasnovaki/orbit-sim/blob/multibody_sim/docs/imgs/Diagram_CoS.png">
</p>

### Quick demo & comments

After launching the nodes with ```roslaunch launch/orbit_sim.launch```, we can perform the following actions:<br>
<br>
<b>1. Adding and removing a spacecraft: </b>by inputting a SC's identifier and its initial state

```console
user@mynotebook:~/ws_catkin/src/orbit_sim$ rosservice call spawncontrol/Spawn -- 1 -15150 0 0 6.2
user@mynotebook:~/ws_catkin/src/orbit_sim$ rosservice call spawncontrol/Spawn -- 2 0 -11000 6.5 -1
user@mynotebook:~/ws_catkin/src/orbit_sim$ rosservice call spawncontrol/Delete 2
user@mynotebook:~/ws_catkin/src/orbit_sim$ rosservice call spawncontrol/Delete 1
```
The green-dashed trajectory is the ellipsis resulted from the orbital elements. However, one can obtain a hyperbolic motion when the initial velocities are too high.

![](https://github.com/lucasnovaki/orbit-sim/blob/multibody_sim/docs/gifs/SpawnerGif_Cropped_2607.gif)

<b>2. Applying thrust: </b>given the tuple $(\Delta{v_{x}}, \Delta{v_{y}})$ defined in the SC's coordinate system, a kick-burn is applied and the body velocity changes at one time step. For a desired thrust only in the direction of motion, keep $\Delta{v_{x}}=0$:

```console
user@mynotebook:~/ws_catkin/src/orbit_sim$ rosservice call spawncontrol/Spawn -- 1 -15150 0 0 6.2
user@mynotebook:~/ws_catkin/src/orbit_sim$ rosservice call Environment/ApplyThrust -- 1 0 0.25
```
![](https://github.com/lucasnovaki/orbit-sim/blob/multibody_sim/docs/gifs/ThrustApo_Cropped_2607.gif)
![](https://github.com/lucasnovaki/orbit-sim/blob/multibody_sim/docs/gifs/ThrustPeri_Cropped_2607.gif)

Here the Oberth Effect becomes visible. The same velocity change $\Delta{v}$ has greater impact on the orbital energy (related to the semi-major axis $a$) when it is applied at high velocities. That is, the periapsis is the most efficient maneuver point to increase the orbital energy with a boost.

<b>3. Performing transfers:</b> given the target orbital elements, a Hohmann Transfer is calculated and if possible executed at the maneuver points.

```console
user@mynotebook:~/ws_catkin/src/orbit_sim$ rosservice call spawncontrol/Spawn -- 1 14000 0 0 -5.9
user@mynotebook:~/ws_catkin/src/orbit_sim$ rosservice call spawncontrol/Spawn -- 2 14000 0 0 -5.9
user@mynotebook:~/ws_catkin/src/orbit_sim$ rosservice call navigation/SetNewOrbit -- 1 11500 0.10 0
user@mynotebook:~/ws_catkin/src/orbit_sim$ rosservice call navigation/SetNewOrbit -- 2 20500 0.15 0
```

![](https://github.com/lucasnovaki/orbit-sim/blob/multibody_sim/docs/gifs/HohmannMultibody_2607v3.gif)

Hohmann transfers are energetically the most efficient solution for the general Lambert's Transfer, although it has some known restrictions: current and target orbits are elliptical, coplanar and don't intersect each other, and their angular momentum have the same direction. Additional restriction: any desired changes in $\omega$ are ignored in this implementation.

In the video above, two SCs start at the same orbit - SC #1 executes a hohmann transfer to an inner orbit, while SC #2 goes to an outer orbit. At the end, the current (green-dashed) and target (pink-dashed) orbits coincide, which indicates a successful maneuver (: