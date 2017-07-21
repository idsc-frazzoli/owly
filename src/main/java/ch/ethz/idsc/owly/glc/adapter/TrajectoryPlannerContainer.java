//code by jl
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.StateSpaceModel;

/** A Container, which contains a {@link TrajectoryPlanner} and its linked objects:
 * Currently:
 * {@link Parameters}
 * {@link StateSpaceModel}
 * @author jolo1992 */
public class TrajectoryPlannerContainer {
  private final TrajectoryPlanner trajectoryPlanner; // TODO maybe PlannerInterface?
  private final Parameters parameters;
  private final StateSpaceModel stateSpaceModel;

  public TrajectoryPlannerContainer(TrajectoryPlanner trajectoryPlanner, Parameters parameters, StateSpaceModel stateSpaceModel) {
    this.trajectoryPlanner = trajectoryPlanner;
    this.parameters = parameters;
    this.stateSpaceModel = stateSpaceModel;
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
}
