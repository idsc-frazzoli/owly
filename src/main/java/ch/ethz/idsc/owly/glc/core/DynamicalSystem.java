// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Ceiling;

public final class DynamicalSystem {
  final Scalar maxTimeStep;
  public DynamicalSystem(Scalar maxTimeStep) {
    this.maxTimeStep=maxTimeStep;
  }
//  public abstract Scalar getMaxTimeStep();

  public Trajectory sim(Integrator integrator, Flow flow, Scalar t0, Scalar tf, Tensor x0) {
    Scalar num_steps = Ceiling.function.apply(tf.subtract(t0).divide(maxTimeStep));
    Scalar dt = tf.subtract(t0).divide(num_steps);
    Trajectory trajectory = new Trajectory();
    StateTime prev = new StateTime(x0, t0);
    trajectory.add(prev);
    for (int c0 = 0; c0 < num_steps.number().intValue(); ++c0) {
      // TODO maybe doesn't add the right number of stateTimes ...
      Tensor x1 = integrator.step(flow, prev.x, dt);
      StateTime next = new StateTime(x1, prev.time.add(dt));
      trajectory.add(next);
      prev = next;
    }
    return trajectory;
  }
}
