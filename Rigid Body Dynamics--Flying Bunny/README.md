# Rigid Body Dynamics--Flying Bunny

![](./Bunny-Rigid.gif)

* At the first place, this a simulation of a rigid Bunny and the collision with walls.
* Rigid Body Collision is detected by capturing the position and velocity of the vertices.
* Collision is responsed by Impulse Method.
* The velocity and position are updated by semi-implicit method (Frog Method).
---
* Exactly, the velocity of collided vertices are converted by friction and collision.
* After that, the impulse is solved out and apply to the whole body to change the linear and angular velocity.
* Finally, new position and rotation are figured out.
