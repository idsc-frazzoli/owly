// code by bapaden, jph, and jl
package ch.ethz.idsc.owly.glc.core;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Tensor;

/** planner is shared between
 * {@link StandardTrajectoryPlanner} and {@link AbstractAnyTrajectoryPlanner} */
/* package */ abstract class AbstractTrajectoryPlanner extends TrajectoryPlanner {
  protected final StateIntegrator stateIntegrator;
  private /* not final */ TrajectoryRegionQuery obstacleQuery;
  private /* not final */ GoalInterface goalInterface;

  protected AbstractTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      TrajectoryRegionQuery obstacleQuery, //
      GoalInterface goalInterface) {
    super(eta);
    GlobalAssert.that(Objects.nonNull(obstacleQuery));
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
  public final GoalInterface getGoalInterface() {
    return goalInterface;
  }

  protected final void setGoalInterface(GoalInterface goalInterface) {
    this.goalInterface = goalInterface;
  }

  protected final void setObstacleQuery(TrajectoryRegionQuery obstacleQuery) {
    this.obstacleQuery = obstacleQuery;
  }

  @Override
  protected Optional<GlcNode> getFurthestGoalNode() {
    return Optional.empty();
  }
}
