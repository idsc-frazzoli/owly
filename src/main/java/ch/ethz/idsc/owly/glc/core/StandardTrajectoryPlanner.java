// code by bapaden, jph, and jl
package ch.ethz.idsc.owly.glc.core;

import java.util.List;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

/** planner is shared between default and abstract-any */
abstract class StandardTrajectoryPlanner extends TrajectoryPlanner {
  final StateIntegrator stateIntegrator;
  final TrajectoryRegionQuery obstacleQuery;
  /* not final */ TrajectoryRegionQuery goalQuery;
  /* not final */ CostFunction costFunction;

  StandardTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      TrajectoryRegionQuery obstacleQuery, //
      TrajectoryRegionQuery goalQuery, //
      CostFunction costFunction //
  ) {
    super(eta);
    this.stateIntegrator = stateIntegrator;
    this.obstacleQuery = obstacleQuery;
    this.goalQuery = goalQuery;
    this.costFunction = costFunction;
  }

  @Override
  public final List<TrajectorySample> detailedTrajectoryTo(GlcNode node) {
    return GlcTrajectories.connect(stateIntegrator, Nodes.listFromRoot(node));
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
  /* package */ final GlcNode createRootNode(Tensor x) {
    return GlcNode.of(null, new StateTime(x, RealScalar.ZERO), RealScalar.ZERO, //
        costFunction.minCostToGoal(x));
  }
}
