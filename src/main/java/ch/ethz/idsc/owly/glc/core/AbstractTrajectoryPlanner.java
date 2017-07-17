// code by bapaden, jph, and jl
package ch.ethz.idsc.owly.glc.core;

import java.util.List;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

/** planner is shared between default and abstract-any */
/* package */ abstract class AbstractTrajectoryPlanner extends TrajectoryPlanner {
  private final StateIntegrator stateIntegrator;
  private final TrajectoryRegionQuery obstacleQuery;
  /* not final */ GoalInterface goalInterface;

  protected AbstractTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      TrajectoryRegionQuery obstacleQuery, //
      GoalInterface goalInterface) {
    super(eta);
    this.stateIntegrator = stateIntegrator;
    this.obstacleQuery = obstacleQuery;
    this.goalInterface = goalInterface;
  }

  @Override
  public final List<TrajectorySample> detailedTrajectoryTo(GlcNode node) {
    return GlcTrajectories.connect(stateIntegrator, Nodes.listFromRoot(node));
  }

  public final StateIntegrator getStateIntegrator() {
    return stateIntegrator;
  }

  @Override
  public final TrajectoryRegionQuery getObstacleQuery() {
    return obstacleQuery;
  }

  @Override
  public final TrajectoryRegionQuery getGoalQuery() {
    return goalInterface;
  }

  @Override
  /* package */ final GlcNode createRootNode(Tensor x) {
    return GlcNode.of(null, new StateTime(x, RealScalar.ZERO), RealScalar.ZERO, //
        goalInterface.minCostToGoal(x));
  }
}
