# ch.ethz.idsc.owly

<a href="https://travis-ci.org/idsc-frazzoli/owly"><img src="https://travis-ci.org/idsc-frazzoli/owly.svg?branch=master" alt="Build Status"></a>

The repository contains Java 8 implementations of motion planners and their variants.

List of algorithms:

* RRT*
* GLC

The references are

* *Sampling-based algorithms for optimal motion planning*
by Sertac Karaman and Emilio Frazzoli,
[IJRR11](http://ares.lids.mit.edu/papers/Karaman.Frazzoli.IJRR11.pdf)
* *A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning*
by Brian Paden and Emilio Frazzoli, 
[arXiv:1607.06966](https://arxiv.org/abs/1607.06966)

The following integrators are available:

* Euler
* Midpoint
* Runge-Kutta 4th order
* Runge-Kutta 5th order 


## Examples

### RRT*

R^2

![r2ani](https://cloud.githubusercontent.com/assets/4012178/26282173/06dccee8-3e0c-11e7-930f-fedab34fe396.gif)

![r2](https://cloud.githubusercontent.com/assets/4012178/26045794/16bd0a54-394c-11e7-9d11-19558bc3be88.png)


### GLC

Pendulum Swing Up

![psu](https://cloud.githubusercontent.com/assets/4012178/25422498/57803d08-2a61-11e7-94c1-87fd1f87e694.png)

---

SE(2)

![se2](https://cloud.githubusercontent.com/assets/4012178/25422502/5b00be4e-2a61-11e7-8798-08fcd8f44658.png)

---

Rice1

![rice1](https://cloud.githubusercontent.com/assets/4012178/25473189/c505917a-2b2e-11e7-8799-0d3bcc32c1d5.png)

---

R^2

![r2](https://cloud.githubusercontent.com/assets/4012178/25473192/c7cdd192-2b2e-11e7-8c9e-72d88d6723d3.png)

---

surface flow in river delta

![lava](https://cloud.githubusercontent.com/assets/4012178/26282194/6855b6d0-3e0c-11e7-92be-cb0ad99e3b8a.gif)

against the direction of current

![delta_c](https://cloud.githubusercontent.com/assets/4012178/26282183/3f750392-3e0c-11e7-95c6-2645545dbbc2.gif)

current reversed

![delta_s](https://cloud.githubusercontent.com/assets/4012178/26282191/59dafa84-3e0c-11e7-9602-2ece6f417bc1.gif)

## References

The repository has over `30` unit tests.