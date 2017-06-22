// code by bapaden, jph, and jl
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class DefaultTrajectoryPlanner extends AbstractStandardTrajectoryPlanner {
  private final Collection<Flow> controls;

  public DefaultTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      TrajectoryRegionQuery obstacleQuery, //
      GoalInterface goalInterface) {
    super(eta, stateIntegrator, obstacleQuery, goalInterface);
    this.controls = controls;
  }

  @Override // from ExpandInterface
  public void expand(final GlcNode node) {
    Map<GlcNode, List<StateTime>> connectors = //
        SharedUtils.integrate(node, controls, getStateIntegrator(), goalInterface);
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
    Optional<GlcNode> optional = getBestOrElsePeek();
    if (optional.isPresent())
      DebugUtils.nodeAmountCheck(optional.get(), node, domainMap().size());
  }

  private void processCandidates( //
      GlcNode node, Map<GlcNode, List<StateTime>> connectors, DomainQueueMap domainQueueMap) {
    for (Entry<Tensor, DomainQueue> entry : domainQueueMap.map.entrySet()) {
      final Tensor domainKey = entry.getKey();
      final DomainQueue domainQueue = entry.getValue();
      if (domainQueueMap != null && !getBest().isPresent()) {
        while (!domainQueue.isEmpty()) {
          final GlcNode next = domainQueue.element();
          final GlcNode formerLabel = getNode(domainKey);
          if (formerLabel != null) {
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              if (getObstacleQuery().isDisjoint(connectors.get(next))) { // no collision
                boolean removed = queue().remove(formerLabel);
                if (!removed)
                  throw new RuntimeException();
                formerLabel.parent().removeEdgeTo(formerLabel);
                node.insertEdgeTo(next);
                boolean replaced = insert(domainKey, next);
                if (!replaced)
                  throw new RuntimeException();
                domainQueue.remove();
                if (!goalInterface.isDisjoint(connectors.get(next)))
                  offerDestination(next);
                break; // leaves the while loop, but not the for loop
              }
            }
          } else {// No formerLabel, so definitely adding a Node
            if (getObstacleQuery().isDisjoint(connectors.get(next))) {
              // removing the nextCandidate from bucket of this domain
              // adding next to tree and DomainMap
              node.insertEdgeTo(next);
              insert(domainKey, next);
              domainQueue.remove();
              // GOAL check
              if (!goalInterface.isDisjoint(connectors.get(next)))
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
  public String infoString() {
    StringBuilder stringBuilder = new StringBuilder(super.infoString() + ", ");
    stringBuilder.append("default...");
    return stringBuilder.toString();
  }
}
