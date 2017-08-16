// code by bapaden, jph, and jl
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** translation of the c++ implementation by bapaden
 * 
 * subsequent modifications include:
 * <ul>
 * <li>parallel integration of trajectories
 * <li>nodes that get replaced in a domain, are also removed from the queue
 * </ul> */
public class StandardTrajectoryPlanner extends AbstractTrajectoryPlanner {
  private final NodeIntegratorFlow nodeIntegratorFlow;

  public StandardTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      TrajectoryRegionQuery obstacleQuery, //
      GoalInterface goalInterface) {
    super(eta, stateIntegrator, obstacleQuery, goalInterface);
    nodeIntegratorFlow = new NodeIntegratorFlow(stateIntegrator, controls);
  }

  @Override // from ExpandInterface
  public void expand(final GlcNode node) {
    Map<GlcNode, List<StateTime>> connectors = nodeIntegratorFlow.parallel(node, getGoalInterface());
    // ---
    DomainQueueMap domainQueueMap = new DomainQueueMap(); // holds candidates for insertion
    for (GlcNode next : connectors.keySet()) { // <- order of keys is non-deterministic
      final Tensor domainKey = convertToKey(next.state());
      final GlcNode former = getNode(domainKey);
      if (Objects.nonNull(former)) {
        // is already some node present from previous exploration ?
        if (Scalars.lessThan(next.merit(), former.merit())) // new node is potentially better than previous one
          domainQueueMap.insert(domainKey, next);
      } else
        domainQueueMap.insert(domainKey, next); // node is considered without comparison to any former node
    }
    processCandidates(node, connectors, domainQueueMap);
  }

  private void processCandidates( //
      GlcNode node, Map<GlcNode, List<StateTime>> connectors, DomainQueueMap domainQueueMap) {
    for (Entry<Tensor, DomainQueue> entry : domainQueueMap.map.entrySet()) {
      final Tensor domainKey = entry.getKey();
      final DomainQueue domainQueue = entry.getValue();
      while (!domainQueue.isEmpty()) {
        final GlcNode next = domainQueue.element();
        final GlcNode formerLabel = getNode(domainKey);
        if (Objects.nonNull(formerLabel)) {
          if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
            if (getObstacleQuery().isDisjoint(connectors.get(next))) { // no collision
              /** removal from queue is unsure; needs to be checked with theory. */
              boolean removed = queue().remove(formerLabel);
              GlobalAssert.that(removed);
              formerLabel.parent().removeEdgeTo(formerLabel);
              node.insertEdgeTo(next);
              boolean replaced = insert(domainKey, next);
              GlobalAssert.that(replaced);
              domainQueue.remove();
              if (!getGoalInterface().isDisjoint(connectors.get(next)))
                offerDestination(next, connectors.get(next));
              // Same principle as in B. Paden's implementation, leaving while loop after first relabel
              break; // leaves the while loop, but not the for loop
            }
          }
        } else { // No formerLabel, so definitely adding a Node
          if (getObstacleQuery().isDisjoint(connectors.get(next))) {
            // removing the nextCandidate from bucket of this domain
            // adding next to tree and DomainMap
            node.insertEdgeTo(next);
            insert(domainKey, next);
            domainQueue.remove();
            // GOAL check
            if (!getGoalInterface().isDisjoint(connectors.get(next)))
              offerDestination(next, connectors.get(next));
            break;
          }
        }
        domainQueue.remove();
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
