// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.List;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

/* package */ abstract class AbstractAnyTrajectoryPlanner extends TrajectoryPlanner {
  private final StateIntegrator stateIntegrator;
  private TrajectoryRegionQuery goalQuery;
  private final TrajectoryRegionQuery obstacleQuery;
  /* not final */ CostFunction costFunction;

  protected AbstractAnyTrajectoryPlanner(Tensor eta, //
      StateIntegrator stateIntegrator, //
      CostFunction costFunction, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery) {
    super(eta);
    this.stateIntegrator = stateIntegrator;
    this.costFunction = costFunction;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
  }

  /** Includes all the functionality of the RootSwitch
   * (deleting of the useless nodes and relabling of modified Domains)
   * @param state the new Rootstate
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  public final int switchRootToState(Tensor state) {
    GlcNode newRoot = this.getNode(convertToKey(state));
    int increaseDepthBy = 0;
    // TODO not nice, as we jump from state to startnode
    if (newRoot != null)
      increaseDepthBy = switchRootToNode(newRoot);
    else {
      System.out.println("This domain is not labelled yet");
      throw new RuntimeException();
      // TODO: should replan everything, as we left old trajectory
    }
    return increaseDepthBy;
  }

  /** Includes all the functionality of the RootSwitch
   * (deleting of the useless nodes and relabling of modified Domains)
   * @param newRoot Node to Switch
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  public abstract int switchRootToNode(GlcNode newRoot);

  @Override
  public final List<TrajectorySample> detailedTrajectoryTo(GlcNode node) {
    return GlcTrajectories.connect(stateIntegrator, Nodes.fromRoot(node));
  }

  @Override
  public final TrajectoryRegionQuery getObstacleQuery() {
    return obstacleQuery;
  }

  @Override
  public final TrajectoryRegionQuery getGoalQuery() {
    return goalQuery;
  }

  @Override
  protected final GlcNode createRootNode(Tensor x) {
    return GlcNode.of(null, new StateTime(x, RealScalar.ZERO), RealScalar.ZERO, //
        costFunction.minCostToGoal(x));
  }
}
