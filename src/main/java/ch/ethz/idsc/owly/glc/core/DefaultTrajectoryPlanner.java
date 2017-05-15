// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;

public class DefaultTrajectoryPlanner extends TrajectoryPlanner {
  private final StateIntegrator stateIntegrator;
  private final Collection<Flow> controls;
  private final CostFunction costFunction;
  private final TrajectoryRegionQuery goalQuery;
  private final TrajectoryRegionQuery obstacleQuery;

  public DefaultTrajectoryPlanner( //
      Tensor partitionScale, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      CostFunction costFunction, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    super(partitionScale);
    this.stateIntegrator = stateIntegrator;
    this.controls = controls;
    this.costFunction = costFunction;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
  }

  @Override
  public void expand(final GlcNode node) {
    // TODO count updates in cell based on costs for benchmarking
    Map<Tensor, DomainQueue> candidates = new HashMap<>();
    Map<GlcNode, List<StateTime>> connectors = new HashMap<>();
    for (final Flow flow : controls) {
      final List<StateTime> trajectory = stateIntegrator.trajectory(node.stateTime(), flow);
      final StateTime last = Trajectories.getLast(trajectory);
      final GlcNode next = new GlcNode(flow, last, //
          node.costFromRoot().add(costFunction.costIncrement(node.stateTime(), trajectory, flow)), //
          costFunction.minCostToGoal(last.x()) //
      );
      connectors.put(next, trajectory);
      // ---
      final Tensor domain_key = convertToKey(next.stateTime().x());
      final GlcNode former = getNode(domain_key);
      if (former != null) { // already some node present from previous exploration
        if (Scalars.lessThan(next.costFromRoot(), former.costFromRoot())) // new node is better than previous one
          if (candidates.containsKey(domain_key))
            candidates.get(domain_key).add(next);
          else
            candidates.put(domain_key, new DomainQueue(next));
      } else
        candidates.put(domain_key, new DomainQueue(next));
    }
    // ---
    processCandidates(node, candidates, connectors);
  }

  private void processCandidates( //
      GlcNode node, Map<Tensor, DomainQueue> candidates, Map<GlcNode, List<StateTime>> connectors) {
    for (Entry<Tensor, DomainQueue> entry : candidates.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final DomainQueue domainQueue = entry.getValue();
      while (!domainQueue.isEmpty()) {
        GlcNode next = domainQueue.poll(); // poll() Retrieves and removes the head of this queue
        if (obstacleQuery.isDisjoint(connectors.get(next))) { // no collision
          node.insertEdgeTo(next);
          insert(domain_key, next);
          if (!goalQuery.isDisjoint(connectors.get(next)))
            offerDestination(next);
          break; // leaves the while loop, but not the for loop
        }
      }
    }
  }

  @Override
  protected GlcNode createRootNode(Tensor x) { // TODO check if time of root node should always be set to 0
    return new GlcNode(null, new StateTime(x, ZeroScalar.get()), ZeroScalar.get(), costFunction.minCostToGoal(x));
  }

  @Override
  public List<StateTime> detailedTrajectoryTo(GlcNode node) {
    return Trajectories.connect(stateIntegrator, Nodes.fromRoot(node));
  }

  @Override
  public TrajectoryRegionQuery getObstacleQuery() {
    return obstacleQuery;
  }

  @Override
  public TrajectoryRegionQuery getGoalQuery() {
    return goalQuery;
  }
}
