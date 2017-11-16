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
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class RnRaytracer {
  private final FixedStateIntegrator stateIntegrator;
  private final TrajectoryRegionQuery trajectoryRegionQuery;
  private final Scalar raySpeed;

  public RnRaytracer(TrajectoryRegionQuery trajectoryRegionQuery, Scalar raySpeed) {
    stateIntegrator = // TODO magic const
        FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RealScalar.of(.05).divide(raySpeed), 200);
    this.trajectoryRegionQuery = trajectoryRegionQuery;
    this.raySpeed = raySpeed;
  }

  /** @param trajectoryRegionQuery
   * @param stateTime
   * @param direction
   * @return */
  public Optional<StateTime> firstMember(StateTime stateTime, Tensor direction) {
    Flow flow = StateSpaceModels.createFlow(SingleIntegratorStateSpaceModel.INSTANCE, direction.multiply(raySpeed));
    return stateIntegrator.firstMember(stateTime, flow, trajectoryRegionQuery);
  }
}
