// code by jph
package ch.ethz.idsc.owly.math.state;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.MidpointIntegrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.TensorRuntimeException;

/** trajectory integration with fixed step size over given time period */
public class FixedStateIntegrator implements StateIntegrator {
  /** uses {@link MidpointIntegrator}
   * 
   * @param timeStep
   * @param trajectorySize
   * @return */
  public static StateIntegrator createDefault(Scalar timeStep, int trajectorySize) {
    return new FixedStateIntegrator(MidpointIntegrator.INSTANCE, timeStep, trajectorySize);
  }

  public static StateIntegrator create(Integrator integrator, Scalar timeStep, int trajectorySize) {
    return new FixedStateIntegrator(integrator, timeStep, trajectorySize);
  }

  private final Integrator integrator;
  private final Scalar timeStep;
  private final int trajectorySize;

  /** @param integrator
   * @param timeStep non-negative period of one step
   * @param trajectorySize number of steps */
  private FixedStateIntegrator(Integrator integrator, Scalar timeStep, int trajectorySize) {
    if (Scalars.lessThan(timeStep, RealScalar.ZERO))
      throw TensorRuntimeException.of(timeStep);
    this.integrator = integrator;
    this.timeStep = timeStep;
    this.trajectorySize = trajectorySize;
  }

  @Override
  public List<StateTime> trajectory(StateTime stateTime, Flow flow) {
    final List<StateTime> trajectory = new ArrayList<>();
    StateTime prev = stateTime;
    for (int count = 0; count < trajectorySize; ++count) {
      StateTime next = new StateTime( //
          integrator.step(flow, prev.x(), timeStep), //
          prev.time().add(timeStep));
      trajectory.add(next);
      prev = next;
    }
    return trajectory;
  }
}
