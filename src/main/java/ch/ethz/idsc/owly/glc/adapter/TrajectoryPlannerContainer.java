package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;

public class TrajectoryPlannerContainer {
  private final TrajectoryPlanner trajectoryPlanner; // TODO maybe PlannerInterface?
  private final Parameters parameters;

  // TODO JONAS if other objects are needed, add
  // --
  public TrajectoryPlannerContainer(TrajectoryPlanner trajectoryPlanner, Parameters parameters) {
    this.trajectoryPlanner = trajectoryPlanner;
    this.parameters = parameters;
  }

  public TrajectoryPlanner getTrajectoryPlanner() {
    return trajectoryPlanner;
  }

  public Parameters getParameters() {
    return parameters;
  }
}
