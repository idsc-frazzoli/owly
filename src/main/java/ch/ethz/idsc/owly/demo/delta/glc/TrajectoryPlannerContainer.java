// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.par.Parameters;
import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.tensor.Tensor;

/** A Container, which contains a {@link TrajectoryPlanner} and its linked objects:
 * Currently:
 * {@link Parameters}
 * {@link StateSpaceModel} */
/* package */ class TrajectoryPlannerContainer {
  private final TrajectoryPlanner trajectoryPlanner;
  private final Parameters parameters;
  private final StateSpaceModel stateSpaceModel;
  private final Region<Tensor> obstacleMap;

  public TrajectoryPlannerContainer( //
      TrajectoryPlanner trajectoryPlanner, //
      Parameters parameters, //
      StateSpaceModel stateSpaceModel, //
      Region<Tensor> obstacleMap) {
    this.trajectoryPlanner = trajectoryPlanner;
    this.parameters = parameters;
    this.stateSpaceModel = stateSpaceModel;
    this.obstacleMap = obstacleMap;
  }

  public TrajectoryPlanner getTrajectoryPlanner() {
    return trajectoryPlanner;
  }

  public Parameters getParameters() {
    return parameters;
  }

  public StateSpaceModel getStateSpaceModel() {
    return stateSpaceModel;
  }

  public Region<Tensor> getObstacleMap() {
    return obstacleMap;
  }
}
