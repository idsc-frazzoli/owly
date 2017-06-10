// code by jl
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
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** An anytime MotionPlanning Algorithm, not optimal/ complete
 * after: [B. Paden] A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning
 * Assumptions: -All states of all obstacles are known at all times
 * -No new Obstacles are discovered */
public class SimpleAnyTrajectoryPlanner extends AbstractAnyTrajectoryPlanner {
  private final Collection<Flow> controls;
  // private final Map<Tensor, DomainQueue> domainCandidateMap = new HashMap<>();

  // private final Queue<Node> queue = new PriorityQueue<>(NodeMeritComparator.instance);
  public SimpleAnyTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      CostFunction costFunction, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    super(eta, stateIntegrator, costFunction, goalQuery, obstacleQuery);
    this.controls = controls;
  }

  @Override // from ExpandInterface
  public void expand(final GlcNode node) {
    // TODO count updates in cell based on costs for benchmarking
    Map<GlcNode, List<StateTime>> connectors = //
        SharedUtils.integrate(node, controls, stateIntegrator, costFunction);
    // Set<Tensor> domainsNeedingUpdate = new HashSet<>();
    // Map<Tensor, DomainQueue> candidates = new HashMap<>();
    CandidatePairQueueMap candidates = new CandidatePairQueueMap();
    for (GlcNode next : connectors.keySet()) { // <- order of keys is non-deterministic
      CandidatePair nextCandidate = new CandidatePair(node, next);
      final Tensor domain_key = convertToKey(next.state());
      candidates.insert(domain_key, nextCandidate);
      // ALL Candidates are saved in CandidateList
    }
    // save candidates in CandidateMap for RootSwitchlater
    processCandidates(node, connectors, candidates);
    nodeAmountCheck(node);
  }

  private void processCandidates( //
      GlcNode node, Map<GlcNode, List<StateTime>> connectors, CandidatePairQueueMap candidates) {
    for (Entry<Tensor, CandidatePairQueue> entry : candidates.map.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final CandidatePairQueue candidateQueue = entry.getValue();
      if (candidateQueue != null && !getBest().isPresent()) {
        while (!candidateQueue.isEmpty()) {
          final CandidatePair nextCandidatePair = candidateQueue.element();
          final GlcNode formerLabel = getNode(domain_key);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (formerLabel != null) {
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              // collision check only if new node is better
              if (obstacleQuery.isDisjoint(connectors.get(next))) {// better node not collision
                // remove former Label from QUEUE
                queue().remove(formerLabel);
                // formerLabel disconnecting
                formerLabel.parent().removeEdgeTo(formerLabel);
                node.insertEdgeTo(next);
                insert(domain_key, next);
                if (!goalQuery.isDisjoint(connectors.get(next)))
                  offerDestination(next);
                candidateQueue.remove();
                break;
              }
            }
            // candidateQueue.remove();
          } else {
            // candidateQueue.remove();
            if (obstacleQuery.isDisjoint(connectors.get(next))) {
              node.insertEdgeTo(next);
              insert(domain_key, next);
              if (!goalQuery.isDisjoint(connectors.get(next)))
                offerDestination(next);
              candidateQueue.remove();
              break;
            }
          }
          candidateQueue.remove();
        }
      }
    }
  }

  @Override
  public int switchRootToNode(GlcNode newRoot) {
    GlcNode oldRoot = Nodes.rootFrom(getBestOrElsePeek());
    if (newRoot.isRoot()) {
      System.out.println("node is already root");
      return 0;
    }
    int increaseDepthBy = newRoot.reCalculateDepth();
    int oldDomainMapSize = domainMap().size();
    int oldQueueSize = queue().size();
    int oldTreeSize = Nodes.ofSubtree(oldRoot).size();
    System.out.println("changing to root:" + newRoot.state());
    // removes the new root from the child list of its parent
    final GlcNode parent = newRoot.parent();
    parent.removeEdgeTo(newRoot);
    Collection<GlcNode> deleteTreeCollection = deleteChildrenOf(oldRoot);
    System.out.println(oldDomainMapSize - domainMap().size() + " out of " + oldDomainMapSize + //
        " Domains removed from DomainMap = " + domainMap().size());
    GlcNode root = Nodes.rootFrom(getBestOrElsePeek());
    System.out.println(deleteTreeCollection.size() + " out of " + oldTreeSize//
        + " Nodes removed from Tree = " + Nodes.ofSubtree(root).size());
    // --
    System.out.println("**Rootswitch finished**");
    return increaseDepthBy;
  }
}