// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Tensor;

public interface AnyPlannerInterface extends ExpandInterface {
  /** Includes all the functionality of the RootSwitch
   * (deleting of the useless nodes and relabeling of modified Domains)
   * @param state the new Rootstate
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  int switchRootToState(Tensor state);

  /** Includes all the functionality of the RootSwitch
   * (deleting of the useless nodes and relabeling of modified Domains)
   * @param newRoot Node to Switch
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  int switchRootToNode(GlcNode newRoot);

  /** Changes the Goal of the current planner:
   * rechecks the tree if goal was found and expanding is needed, updates Merit of Nodes in Queue, relabels domains
   * @param newGoal, new Goalmanager
   * @param goalCheckHelp, which includes all Nodes, which could have a trajectory in the Goal
   * @return boolean, true if Goal was already found in oldTree */
  boolean changeToGoal(GoalInterface newGoal, Region goalCheckHelp);

  /** Changes the Goal of the current planner:
   * rechecks the tree if Goal was found and expanding is needed, updates Merit of Nodes in Queue, relabels domains
   * @param newGoal, new Goalmanager
   * @return boolean, true if Goal was already found in oldTree */
  boolean changeToGoal(final GoalInterface newGoal);

  /** Updates the tree with the new Obstacle Information
   * 
   * @param newObstacle the new Query for the new Obstacle Information */
  void ObstacleUpdate(TrajectoryRegionQuery newObstacle);

  /** @return the StateTime Trajectory to the best Goal, or if no goal was found NULL */
  List<StateTime> trajectoryToBest();

  /** The TrajectorySample Trajectory is more detailed and also includes the Input used.
   * @return TrajectorySample Trajectory to the best Goal or if no goal was found NULL */
  List<TrajectorySample> detailedTrajectoryToBest();

  /** @return Returns the StateTime, which is the furthest in the GoalRegion,
   * Furthest is determined by the merit of the Node at the end of its trajectory */
  Optional<StateTime> getFurthestGoalState();

  /** @return the node, to which the trajectory should lead:
   * The Furthest, the best or the top of the Queue */
  Optional<GlcNode> getFinalGoalNode();
}
