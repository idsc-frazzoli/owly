package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.StateSpaceModel;

public class TrajectoryPlannerContainer {
  private final TrajectoryPlanner trajectoryPlanner; // TODO maybe PlannerInterface?
  private final Parameters parameters;
  private final StateSpaceModel stateSpaceModel;

  // TODO JAN Generic types? as different subclasses of trajectoryPlanners or parameters exist
  // TODO JONAS if other objects are needed, add
  // --
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
