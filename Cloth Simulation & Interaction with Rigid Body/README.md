# Cloth Simulation & Interaction with Rigid Body

![](Cloth.gif)

* Above all, this is a simulation of a piece of cloth which could move and deform.
* What's more both of them are based on Mass-Spring System and the Triangle Mesh.

There are two methods applied to it.

* For stability, Implicit Integration method is chosen in stead of Explicit Integration.
* To solve for the expected position after every step, Newton-Raphson Method is applied for optimization.
* To solve the Linear equations, Jacobi method with Chebyshev Acceleration is implemented.

* As an alternative way, PBD method (position based dynamics) is also used.
* To make the result more accurate, the method is in the Jacobi style instead of Gauss-Seidel stlye.
