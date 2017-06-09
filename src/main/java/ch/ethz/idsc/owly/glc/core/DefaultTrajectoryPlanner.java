// code by bapaden, jph, and jl
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class DefaultTrajectoryPlanner extends TrajectoryPlanner {
  private final StateIntegrator stateIntegrator;
  private final Collection<Flow> controls;
  private final CostFunction costFunction;
  private final TrajectoryRegionQuery goalQuery;
  private final TrajectoryRegionQuery obstacleQuery;

  public DefaultTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      CostFunction costFunction, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    super(eta);
    this.stateIntegrator = stateIntegrator;
    this.controls = controls;
    this.costFunction = costFunction;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
  }

  @Override // from ExpandInterface
  public void expand(final GlcNode node) {
    Map<GlcNode, List<StateTime>> connectors = //
        SharedUtils.integrate(node, controls, stateIntegrator, costFunction);
    // ---
    DomainQueueMap domainQueueMap = new DomainQueueMap(); // holds candidates from insertion
    for (GlcNode next : connectors.keySet()) { // <- order of keys is non-deterministic
      final Tensor domainKey = convertToKey(next.state());
      final GlcNode former = getNode(domainKey);
      if (former != null) {
        // is already some node present from previous exploration ?
        if (Scalars.lessThan(next.merit(), former.merit())) // new node is potentially better than previous one
          domainQueueMap.insert(domainKey, next);
      } else
        domainQueueMap.insert(domainKey, next); // node is considered without comparison to any former node
    }
    processCandidates(node, connectors, domainQueueMap);
    nodeAmountCheck(node);
  }

  private void processCandidates( //
      GlcNode node, Map<GlcNode, List<StateTime>> connectors, DomainQueueMap domainQueueMap) {
    for (Entry<Tensor, DomainQueue> entry : domainQueueMap.map.entrySet()) {
      final Tensor domainKey = entry.getKey();
      final DomainQueue domainQueue = entry.getValue();
      if (domainQueueMap != null && best == null) {
        while (!domainQueue.isEmpty()) {
          final GlcNode next = domainQueue.element();
          final GlcNode formerLabel = getNode(domainKey);
          if (formerLabel != null) {
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              if (obstacleQuery.isDisjoint(connectors.get(next))) { // no collision
                queue().remove(formerLabel);
                formerLabel.parent().removeEdgeTo(formerLabel);
                node.insertEdgeTo(next);
                boolean replaced = insert(domainKey, next);
                if (!replaced)
                  throw new RuntimeException();
                domainQueue.remove();
                if (!goalQuery.isDisjoint(connectors.get(next)))
                  offerDestination(next);
                break; // leaves the while loop, but not the for loop
              }
            }
          } else {// No formerLabel, so definitely adding a Node
            if (obstacleQuery.isDisjoint(connectors.get(next))) {
              // removing the nextCandidate from bucket of this domain
              // adding next to tree and DomainMap
              node.insertEdgeTo(next);
              insert(domainKey, next);
              domainQueue.remove();
              // GOAL check
              if (!goalQuery.isDisjoint(connectors.get(next)))
                offerDestination(next);
              break;
            }
          }
          domainQueue.remove();
        }
      }
    }
  }

  @Override
  protected GlcNode createRootNode(Tensor x) {
    return GlcNode.of(null, new StateTime(x, RealScalar.ZERO), RealScalar.ZERO, costFunction.minCostToGoal(x));
  }

  @Override
  public List<TrajectorySample> detailedTrajectoryTo(GlcNode node) {
    return GlcTrajectories.connect(stateIntegrator, Nodes.fromRoot(node));
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
