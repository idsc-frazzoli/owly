// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.Tensor;

public abstract class DynamicalSystem {
  public abstract Tensor flow(Tensor x, Tensor u);

  public abstract Tensor step(Tensor x, Tensor u, double dt);

  public abstract double getLipschitz();

  public abstract double getMaxTimeStep();

  public Trajectory sim(double t0, double tf, Tensor x0, Tensor u) {
    double num_steps = Math.ceil((tf - t0) / getMaxTimeStep());
    double dt = (tf - t0) / num_steps;
    Trajectory trajectory = new Trajectory();
    StateTime prev = new StateTime(x0, t0);
    trajectory.add(prev);
    for (int c0 = 0; c0 < num_steps; ++c0) {
      // TODO maybe doesn't add the right number of stateTimes ...
      Tensor x1 = step(prev.tensor, u, dt);
      StateTime next = new StateTime(x1, prev.time + dt);
      trajectory.add(next);
      prev = next;
    }
    return trajectory;
  }
}
