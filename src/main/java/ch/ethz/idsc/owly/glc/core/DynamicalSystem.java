// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Ceiling;

public abstract class DynamicalSystem {
  public abstract Tensor flow(Tensor x, Tensor u);

  public abstract Tensor step(Tensor x, Tensor u, Scalar dt);

  public abstract Scalar getLipschitz();

  public abstract Scalar getMaxTimeStep();

  public Trajectory sim(Scalar t0, Scalar tf, Tensor x0, Tensor u) {
    Scalar num_steps = Ceiling.function.apply(tf.subtract(t0).divide(getMaxTimeStep()));
    Scalar dt = tf.subtract(t0).divide(num_steps);
    Trajectory trajectory = new Trajectory();
    StateTime prev = new StateTime(x0, t0);
    trajectory.add(prev);
    for (int c0 = 0; c0 < num_steps.number().intValue(); ++c0) {
      // TODO maybe doesn't add the right number of stateTimes ...
      Tensor x1 = step(prev.tensor, u, dt);
      StateTime next = new StateTime(x1, prev.time.add(dt));
      trajectory.add(next);
      prev = next;
    }
    return trajectory;
  }
}
