// code by bapaden, jph, and jl
package ch.ethz.idsc.owl.glc.core;

import java.io.Serializable;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.NavigableMap;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.TreeMap;

import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.owl.glc.adapter.GlcNodes;
import ch.ethz.idsc.owl.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owl.glc.any.AbstractAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.std.PlannerConstraint;
import ch.ethz.idsc.owl.math.StateTimeTensorFunction;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Floor;

/** base class for generalized label correction implementation */
public abstract class TrajectoryPlanner implements ExpandInterface<GlcNode>, Serializable {
  private final Tensor eta;
  // ---
  private final Queue<GlcNode> queue = new PriorityQueue<>(NodeMeritComparator.INSTANCE);
  private final Map<Tensor, GlcNode> domainMap = new HashMap<>();
  /** best is a reference to a Node in the goal region,
   * or null if such a node has not been identified
   * use function setBestNull() to reset best to null */
  protected final NavigableMap<GlcNode, List<StateTime>> best = //
      new TreeMap<>(NodeMeritComparator.INSTANCE);
  private int replaceCount = 0;

  /* package */ TrajectoryPlanner(Tensor eta) {
    this.eta = eta.copy().unmodifiable();
  }

  /** @return eta as unmodifiable tensor */
  public final Tensor getEta() {
    return eta;
  }

  /** mapping from state time to domain coordinates
   * 
   * <p>The default value drops time information and only considers
   * {@link StateTime#state()}.
   * 
   * Examples: identity, mod, log, ... */
  public StateTimeTensorFunction represent = StateTime::state;

  /** Floor(eta * state) == Floor(state / domain_size)
   * 
   * @param stateTime
   * @return */
  protected Tensor convertToKey(StateTime stateTime) {
    return eta.pmul(represent.apply(stateTime)).map(Floor.FUNCTION);
  }

  /** @param stateTime */
  public final void insertRoot(StateTime stateTime) {
    GlobalAssert.that(queue.isEmpty() && domainMap.isEmpty()); // root insertion requires empty planner
    boolean replaced = insert(convertToKey(stateTime), GlcNodes.createRoot(stateTime, getGoalInterface()));
    GlobalAssert.that(!replaced); // root insertion should not replace any other node
  }

  /** @param domain_key
   * @param node
   * @return true if node replaces a existing entry in the domain map,
   * false if the domain map did not have a pre-existing mapping from given domain_key */
  protected final boolean insert(Tensor domain_key, GlcNode node) {
    if (!node.isLeaf()) {
      System.err.println("The Inserted Node has children");
      throw new RuntimeException();
    }
    queue.add(node);
    final boolean replaced = domainMap.containsKey(domain_key);
    domainMap.put(domain_key, node);
    if (replaced)
      ++replaceCount;
    return replaced;
  }

  // /** @param domain_key
  // * @return node in domain or null if domain has not been assigned a node yet */
  // protected final GlcNode getNode(Tensor domain_key) {
  // return domainMap.get(domain_key);
  // }
  /** @param domain_key
   * @return node in domain or Optional.empty() if domain has not been assigned a node yet */
  protected final Optional<GlcNode> getNode(Tensor domain_key) {
    return Optional.ofNullable(domainMap.get(domain_key));
  }

  @Override // from ExpandInterface
  public final Optional<GlcNode> pollNext() {
    return Optional.ofNullable(queue.poll()); // Queue#poll() returns the head of queue, or null if queue is empty
  }

  /** method is invoked to notify planner that the
   * intersection of the goal interface and the connector is non-empty
   * 
   * {@link AbstractAnyTrajectoryPlanner} overrides this method
   * 
   * access to method is 'synchronized' to make modification of
   * data structure thread safe.
   * 
   * @param node
   * @param connector */
  protected synchronized void offerDestination(GlcNode node, List<StateTime> connector) {
    best.put(node, connector);
    if (1 < best.size()) {
      best.remove(best.lastKey());
      GlobalAssert.that(best.size() == 1);
    }
  }

  @Override // from ExpandInterface
  public final Optional<GlcNode> getBest() {
    return Optional.ofNullable(best.isEmpty() ? null : best.firstKey());
  }

  protected final void setBestNull() {
    best.clear();
  }

  /** @return best node known to be in goal, or top node in queue, or null,
   * in this order depending on existence */
  public final Optional<GlcNode> getBestOrElsePeek() {
    // Queue#peek() returns the head of queue, or null if queue is empty
    return Optional.ofNullable(getBest().orElse(queue.peek()));
  }

  /** @return number of replacements in the domain map caused by {@link TrajectoryPlanner#insert(Tensor, GlcNode)} */
  public final int replaceCount() {
    return replaceCount;
  }

  /** @return state integrator for the state space to generate trajectories from given controls */
  public abstract StateIntegrator getStateIntegrator();

  /** @return obstacle query for the purpose of inspection, i.e. no alteration should be made */
  public abstract PlannerConstraint getPlannerConstraint();

  /** @return goal query for the purpose of inspection, i.e. no alteration should be made */
  public abstract GoalInterface getGoalInterface();

  protected final Collection<GlcNode> queue() {
    return queue;
  }

  protected final Map<Tensor, GlcNode> domainMap() {
    return domainMap;
  }

  /** @return unmodifiable view on queue for display and tests */
  public final Collection<GlcNode> getQueue() {
    return Collections.unmodifiableCollection(queue);
  }

  /** @return unmodifiable view on domain map for display and tests */
  public final Map<Tensor, GlcNode> getDomainMap() {
    return Collections.unmodifiableMap(domainMap);
  }

  public String infoString() {
    StringBuilder stringBuilder = new StringBuilder();
    stringBuilder.append("nodes:" + domainMap().values().size() + ", ");
    {
      // FIXME
      // TrajectoryRegionQuery trajectoryRegionQuery = getObstacleQuery();
      // if (trajectoryRegionQuery instanceof StateTimeCollector) {
      // Collection<StateTime> collection = ((StateTimeCollector) trajectoryRegionQuery).getMembers();
      // stringBuilder.append("obstacles:" + collection.size() + ", ");
      // }
    }
    stringBuilder.append("replacements:" + replaceCount());
    return stringBuilder.toString();
  }

  /** @return the node, to which the trajectory should lead:
   * The Furthest, the best or the top of the Queue */
  public Optional<GlcNode> getFinalGoalNode() {
    if ((getGoalInterface() instanceof TrajectoryGoalMarker)) {
      Optional<GlcNode> furthest = getFurthestGoalNode();
      if (furthest.isPresent())
        return furthest;
      // if TrajectoryGoalMarker is used, the heuristic will not be "good enough for guidance"
      return getBest();
    }
    return HeuristicQ.of(getGoalInterface()) //
        ? getBestOrElsePeek() //
        : getBest();
  }

  /** @param stateTime to be checked if corresponds to an existing Node in tree
   * @return the Optional Node if it exist, if not empty */
  public Optional<GlcNode> existsInTree(StateTime stateTime) {
    throw new RuntimeException();
  }

  /** @return Returns the Node, which belongs to the furthest StateTime in the GoalRegion,
   * Furthest is determined by the best node in the highest list entry of the Goallist
   * or Optional.empty() if no such node has been identified yet */
  protected final Optional<GlcNode> getFurthestGoalNode() {
    if (!(getGoalInterface() instanceof TrajectoryGoalManager))
      throw new RuntimeException(); // can only run on for TrajectoryGoalManager
    List<Region<Tensor>> goalRegions = ((TrajectoryGoalManager) getGoalInterface()).getGoalRegionList();
    ListIterator<Region<Tensor>> iter = goalRegions.listIterator(goalRegions.size());
    DomainQueue regionQueue = new DomainQueue(); // priority queue over merit of GlcNodes
    while (iter.hasPrevious()) { // goes through all regions from last to first
      TrajectoryRegionQuery strq = SimpleTrajectoryRegionQuery.timeInvariant(iter.previous());// go through Regions from the last to first:
      for (GlcNode tempBest : best.keySet()) {
        List<StateTime> trajectory = best.get(tempBest);
        Optional<StateTime> optional = strq.firstMember(trajectory);
        if (optional.isPresent())
          regionQueue.add(tempBest); // saves members in PQ
      }
      if (!regionQueue.isEmpty())
        break; // leave while loop when Nodes where found in latest Goalregion
    }
    return Optional.ofNullable(regionQueue.peek());
  }
}
