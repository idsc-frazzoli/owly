// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.Optional;

import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

public class RnRaytracer {
  private final FixedStateIntegrator stateIntegrator = // TODO magic const
      FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RealScalar.of(.1), 100);
  private final TrajectoryRegionQuery trajectoryRegionQuery;

  public RnRaytracer(TrajectoryRegionQuery trajectoryRegionQuery) {
    this.trajectoryRegionQuery = trajectoryRegionQuery;
  }

  /** @param trajectoryRegionQuery
   * @param stateTime
   * @param direction
   * @return */
  public Optional<StateTime> firstMember(StateTime stateTime, Tensor direction) {
    Flow flow = StateSpaceModels.createFlow(SingleIntegratorStateSpaceModel.INSTANCE, direction);
    return stateIntegrator.firstMember(stateTime, flow, trajectoryRegionQuery);
  }
}
