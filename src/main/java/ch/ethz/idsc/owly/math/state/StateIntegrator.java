// code by jph
package ch.ethz.idsc.owly.math.state;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.MidpointIntegrator;
import ch.ethz.idsc.tensor.Scalar;

public class StateIntegrator {
  public static StateIntegrator createDefault(Scalar timeStep, int trajectorySize) {
    return new StateIntegrator(new MidpointIntegrator(), timeStep, trajectorySize);
  }

  public static StateIntegrator create(Integrator integrator, Scalar timeStep, int trajectorySize) {
    return new StateIntegrator(integrator, timeStep, trajectorySize);
  }

  private final Integrator integrator;
  private final Scalar timeStep;
  private final int trajectorySize;

  /** constructor is private
   * constructor is invoked from static functions
   * 
   * @param integrator
   * @param timeStep
   * @param trajectorySize */
  private StateIntegrator(Integrator integrator, Scalar timeStep, int trajectorySize) {
    this.integrator = integrator;
    this.timeStep = timeStep;
    this.trajectorySize = trajectorySize;
  }

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
