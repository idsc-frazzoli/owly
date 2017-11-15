// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

public class Raytracer {
  private final StateIntegrator stateIntegrator = // TODO magic const
      FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RealScalar.of(.05), 20 * 10);
  private final TrajectoryRegionQuery trajectoryRegionQuery;

  public Raytracer(TrajectoryRegionQuery trajectoryRegionQuery) {
    this.trajectoryRegionQuery = trajectoryRegionQuery;
  }

  /** @param trajectoryRegionQuery
   * @param origin
   * @param direction
   * @return */
  public Optional<StateTime> firstMember(Tensor origin, Tensor direction) {
    Flow flow = StateSpaceModels.createFlow(SingleIntegratorStateSpaceModel.INSTANCE, direction);
    // TODO magic const
    // TODO JAN integrate only until collision
    List<StateTime> trajectory = stateIntegrator.trajectory(new StateTime(origin, RealScalar.ZERO), flow);
    return trajectoryRegionQuery.firstMember(trajectory);
  }
}
