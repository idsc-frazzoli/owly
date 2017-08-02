// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Tensor;

public interface AnyPlannerInterface extends ExpandInterface {
  /** Includes all the functionality of the RootSwitch
   * (deleting of the useless nodes and relabeling of modified Domains)
   * @param state the new Rootstate
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  int switchRootToState(Tensor state);

  /** Includes all the functionality of the RootSwitch
   * (deleting of the useless nodes and relabling of modified Domains)
   * @param newRoot Node to Switch
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  int switchRootToNode(GlcNode newRoot);

  /** Changes the Goal of the current planner:
   * rechecks the tree if expanding is needed, updates Merit of Nodes in Queue
   * @param newCostFunction modified Costfunction for heuristic
   * @param newGoal New GoalRegion
   * @return boolean, true if Goal was already found in oldTree */
  boolean changeToGoal(final GoalInterface newGoal);

  /** @return the StateTime Trajectory to the best Goal, or if no goal was found NULL */
  List<StateTime> trajectoryToBest();

  /** The TrajectorySample Trajectory is more detailed and also includes the Input used.
   * @return TrajectorySample Trajectory to the best Goal or if no goal was found NULL */
  List<TrajectorySample> detailedTrajectoryToBest();

  /** @return first State in goal region from furthest Node, or Optional.empty() if no such node has been identified yet */
  Optional<StateTime> getFurthestGoalState(List<Region> goalRegions);

  /** @return furthest Node (lowest cost in higehst listindex), whose incoming trajectory is in GoalRegion,
   * or Optional.empty() if no such node has been identified yet */
  Optional<GlcNode> getFurthestGoalNode(List<Region> goalRegions);
}
