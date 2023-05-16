# Elastic Body Simulation -- Bouncy House

![](House-bouncy.gif)

* First of all, it is a simulation of an Elastic Body with divided grid on it.
* The forces are calculated by integrating the Cauchy Stress Tensor and Normal Direction of the current state.
* For convenience, both of them are converted to First Piola–Kirchhoff stress and Normal Direction of the reference state.
* As for the formula to calculate the former, the St. Venant-Kirchhoff Model is selected.
* FEM/FVM Framework is the base.
