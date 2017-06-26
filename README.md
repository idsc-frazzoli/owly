# ch.ethz.idsc.owly

<a href="https://travis-ci.org/idsc-frazzoli/owly"><img src="https://travis-ci.org/idsc-frazzoli/owly.svg?branch=master" alt="Build Status"></a>

The repository contains Java 8 implementations of motion planners and their variants.

List of algorithms:

* GLC
* RRT*

The references are

* *A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning*
by Brian Paden and Emilio Frazzoli, 
[arXiv:1607.06966](https://arxiv.org/abs/1607.06966)
* *Sampling-based algorithms for optimal motion planning*
by Sertac Karaman and Emilio Frazzoli,
[IJRR11](http://ares.lids.mit.edu/papers/Karaman.Frazzoli.IJRR11.pdf)

The following integrators are available:

* Euler, Midpoint
* Runge-Kutta 4th order, and 5th order 

The `owly` repository implements visualizations in 2D as showcased below.

The separate repository [owly3d](https://github.com/idsc-frazzoli/owly3d) implements animations and visualizations in 3D.


## Examples

### GLC

Pendulum Swing Up

![owly_psu1](https://user-images.githubusercontent.com/4012178/27012135-8979aae6-4eca-11e7-815e-95dd9b9ee0ea.png)

---

SE(2)

![se2](https://cloud.githubusercontent.com/assets/4012178/25422502/5b00be4e-2a61-11e7-8798-08fcd8f44658.png)

---

Rice1

![owly_rice1](https://user-images.githubusercontent.com/4012178/27012136-8979beaa-4eca-11e7-880f-7274c807c2b8.png)

---

R^2

![r2](https://cloud.githubusercontent.com/assets/4012178/25473192/c7cdd192-2b2e-11e7-8c9e-72d88d6723d3.png)

![owly_r2sphere](https://user-images.githubusercontent.com/4012178/27012137-897c3702-4eca-11e7-9665-72ffb87136ac.png)

---

surface flow in river delta

![lava](https://cloud.githubusercontent.com/assets/4012178/26282194/6855b6d0-3e0c-11e7-92be-cb0ad99e3b8a.gif)

against the direction of current

![delta_c](https://cloud.githubusercontent.com/assets/4012178/26282183/3f750392-3e0c-11e7-95c6-2645545dbbc2.gif)

current reversed

![delta_s](https://cloud.githubusercontent.com/assets/4012178/26282191/59dafa84-3e0c-11e7-9602-2ece6f417bc1.gif)

### AnyTime GLC

R^2

![r2_circle_gif](https://user-images.githubusercontent.com/6703495/27226674-6a78071c-52a0-11e7-948c-7a12af42a7c1.gif)

### RRT*

R^2

![r2ani](https://cloud.githubusercontent.com/assets/4012178/26282173/06dccee8-3e0c-11e7-930f-fedab34fe396.gif)

![r2](https://cloud.githubusercontent.com/assets/4012178/26045794/16bd0a54-394c-11e7-9d11-19558bc3be88.png)

## References

The repository has over `60` unit tests.
