// code by jl
package ch.ethz.idsc.owl.glc.any;

import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.owl.data.tree.Nodes;
import ch.ethz.idsc.owl.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.InvertedRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.Regions;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

// TODO: test changes marked with "YNAGER"
/** An anytime MotionPlanning Algorithm, not optimal/ complete
 * after: [B. Paden] A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning
 * Assumptions: -All states of all obstacles are known at all times
 * -No new Obstacles are discovered */
public class SimpleAnyTrajectoryPlanner extends AbstractAnyTrajectoryPlanner {
  public SimpleAnyTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      TrajectoryRegionQuery obstacleQuery, //
      GoalInterface goalInterface //
  ) {
    super(eta, stateIntegrator, controls, obstacleQuery, goalInterface);
  }

  @Override // from ExpandInterface
  public void expand(final GlcNode node) {
    Map<GlcNode, List<StateTime>> connectors = controlsIntegrator.from(node);
    // --
    CandidatePairQueueMap candidates = new CandidatePairQueueMap();
    for (GlcNode next : connectors.keySet()) { // <- order of keys is non-deterministic
      CandidatePair nextCandidate = new CandidatePair(node, next);
      final Tensor domain_key = convertToKey(next.stateTime());
      candidates.insert(domain_key, nextCandidate);
      // ALL Candidates are saved in CandidateList
    }
    // save candidates in CandidateMap for RootSwitchlater
    processCandidates(node, connectors, candidates);
    StaticHelper.nodeAmountCheck(getBestOrElsePeek().get(), node, domainMap().size());
  }

  private void processCandidates( //
      GlcNode nextParent, Map<GlcNode, List<StateTime>> connectors, CandidatePairQueueMap candidates) {
    for (Entry<Tensor, CandidatePairQueue> entry : candidates.map.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final CandidatePairQueue candidateQueue = entry.getValue();
      if (Objects.nonNull(candidateQueue) && !getBest().isPresent()) {
        while (!candidateQueue.isEmpty()) {
          final CandidatePair nextCandidatePair = candidateQueue.element();
          final Optional<GlcNode> formerLabelOpt = getNode(domain_key);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (formerLabelOpt.isPresent()) {
            GlcNode formerLabel = formerLabelOpt.get();
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              // collision check only if new node is better
              // if (!getObstacleQuery().firstMember(connectors.get(next)).isPresent()) {// better node not collision
              if (getPlannerConstraint().isSatisfied(nextParent, connectors.get(next), next.flow())) { //  YNAGER
                // remove former Label from QUEUE
                final Collection<GlcNode> subDeleteTree = deleteSubtreeOf(formerLabel);
                if (subDeleteTree.size() > 1)
                  System.err.println("Pruned Tree of Size: " + subDeleteTree.size());
                // formerLabel disconnecting
                formerLabel.parent().removeEdgeTo(formerLabel);
                insertNodeInTree(nextParent, next);
                if (isInsideGoal(connectors.get(next)))
                  offerDestination(next, connectors.get(next));
                candidateQueue.remove();
                break;
              }
            }
          } else {
            // if (!getObstacleQuery().firstMember(connectors.get(next)).isPresent()) {
            if (getPlannerConstraint().isSatisfied(nextParent, connectors.get(next), next.flow())) { //  YNAGER
              nextParent.insertEdgeTo(next);
              insert(domain_key, next);
              if (isInsideGoal(connectors.get(next)))
                offerDestination(next, connectors.get(next));
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
    GlcNode oldRoot = Nodes.rootFrom(getBestOrElsePeek().get());
    if (newRoot.isRoot()) {
      System.out.println("node is already root");
      return 0;
    }
    int increaseDepthBy = newRoot.depthDifferenceToRoot();
    int oldDomainMapSize = domainMap().size();
    int oldTreeSize = Nodes.ofSubtree(oldRoot).size();
    System.out.println("changing to root:" + newRoot.state());
    // removes the new root from the child list of its parent
    final GlcNode parent = newRoot.parent();
    parent.removeEdgeTo(newRoot);
    Collection<GlcNode> deleteTreeCollection = deleteSubtreeOf(oldRoot);
    System.out.println(oldDomainMapSize - domainMap().size() + " out of " + oldDomainMapSize + //
        " Domains removed from DomainMap = " + domainMap().size());
    GlcNode root = Nodes.rootFrom(getBestOrElsePeek().get());
    System.out.println(deleteTreeCollection.size() + " out of " + oldTreeSize//
        + " Nodes removed from Tree = " + Nodes.ofSubtree(root).size());
    // --
    System.out.println("**Rootswitch finished**");
    return increaseDepthBy;
  }

  @Override
  public String infoString() {
    StringBuilder stringBuilder = new StringBuilder(super.infoString() + ", ");
    stringBuilder.append("SimpleAnyPlanner");
    if (getGoalInterface() instanceof TrajectoryGoalManager)
      stringBuilder.append(", with a TrajectoryGoalManger");
    return stringBuilder.toString();
  }

  @Override
  void relabelingDomains() {
    // nothing needs to be done, as Optimality is not kept
  }

  /** Only checking the Nodes of the tree, not the trajectory */
  @Override
  boolean goalCheckTree(final Region<Tensor> goalCheckTree) {
    final Collection<GlcNode> treeCollection = Nodes.ofSubtree(getRoot());
    treeCollection.parallelStream().forEach(node -> {
      if (getGoalInterface().isMember(node.stateTime()))
        offerDestination(node, Arrays.asList(node.stateTime()));
    });
    return getBest().isPresent();
  }

  @Override
  public void obstacleUpdate(TrajectoryRegionQuery newObstacle) {
    obstacleUpdate(newObstacle, new InvertedRegion(Regions.emptyRegion()));
  }

  @Override
  public void obstacleUpdate(TrajectoryRegionQuery newObstacle, Region<Tensor> possibleNewObstacleRegion) {
    throw new RuntimeException(); // TODO JONAS implement
  }
}