// code by bapaden, jph, and jl
package ch.ethz.idsc.owly.glc.core;

import java.io.Serializable;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.TreeMap;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeCollector;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
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
  /* package */ final NavigableMap<GlcNode, List<StateTime>> best = new TreeMap<>(NodeMeritComparator.INSTANCE);
  private int replaceCount = 0;

  /* package */ TrajectoryPlanner(Tensor eta) {
    this.eta = eta.copy().unmodifiable();
  }

  /** @return eta as unmodifiable tensor */
  public final Tensor getEta() {
    return eta;
  }

  /** mapping from state to domain coordinates
   * Examples: identity, mod, log, ... */
  public TensorUnaryOperator represent = tensor -> tensor;

  /** Floor(eta * state) == Floor(state / domain_size)
   * 
   * @param x state
   * @return */
  /* package */ Tensor convertToKey(Tensor x) {
    return eta.pmul(represent.apply(x)).map(Floor.FUNCTION);
  }

  /** @param stateTime */
  public final void insertRoot(StateTime stateTime) {
    GlobalAssert.that(queue.isEmpty() && domainMap.isEmpty()); // root insertion requires empty planner
    boolean replaced = insert(convertToKey(stateTime.state()), GlcNodes.createRoot(stateTime, getGoalInterface()));
    GlobalAssert.that(!replaced); // root insertion should not replace any other node
  }

  /** this function may be deprecated in the future.
   * in new applications use {@link #insertRoot(StateTime)} instead */
  public final void insertRoot(Tensor x) {
    insertRoot(new StateTime(x, RealScalar.ZERO));
  }

  /** @param domain_key
   * @param node
   * @return true if node replaces a existing entry in the domain map,
   * false if the domain map did not have a pre-existing mapping from given domain_key */
  /* package */ final boolean insert(Tensor domain_key, GlcNode node) {
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

  /** @param domain_key
   * @return node in domain or null if domain has not been assigned a node yet */
  /* package */ final GlcNode getNode(Tensor domain_key) {
    return domainMap.get(domain_key);
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
    while (1 < best.size()) // `if` should be sufficient, but `while` to be sure
      best.remove(best.lastKey());
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

  /** @param node
   * @return densely sampled trajectory from root to given node
   * that is the result of integrating the flows between the nodes */
  public abstract List<TrajectorySample> detailedTrajectoryTo(GlcNode node);

  /** @return obstacle query for the purpose of inspection, i.e. no alteration should be made */
  public abstract TrajectoryRegionQuery getObstacleQuery();

  /** @return goal query for the purpose of inspection, i.e. no alteration should be made */
  public abstract GoalInterface getGoalInterface();

  /* package */ final Collection<GlcNode> queue() {
    return queue;
  }

  /* package */ final Map<Tensor, GlcNode> domainMap() {
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
      TrajectoryRegionQuery trajectoryRegionQuery = getObstacleQuery();
      if (trajectoryRegionQuery instanceof StateTimeCollector) {
        Collection<StateTime> collection = ((StateTimeCollector) trajectoryRegionQuery).getMembers();
        stringBuilder.append("obstacles:" + collection.size() + ", ");
      }
    }
    stringBuilder.append("replacements:" + replaceCount());
    return stringBuilder.toString();
  }

  /** @return the node, to which the trajectory should lead:
   * The Furthest, the best or the top of the Queue */
  public Optional<GlcNode> getFinalGoalNode() {
    if ((getGoalInterface() instanceof TrajectoryGoalManager)) {
      Optional<GlcNode> furthest = getFurthestGoalNode();
      if (furthest.isPresent())
        return furthest;
      else // if TrajectoryGoalManager is used, the heuristic will not be "good enough for guidance"
        return getBest();
    }
    if (HeuristicQ.of(getGoalInterface())) {
      return getBestOrElsePeek();
    }
    return getBest();
  }

  /** @param stateTime to be checked if corresponds to an existing Node in tree
   * @return the Optional Node if it exist, if not empty */
  public Optional<GlcNode> existsInTree(StateTime stateTime) {
    throw new RuntimeException();
  }

  // TODO assuming that no TrajectoryGoalManager is used in standardplanner.
  // Does this make sense?
  /** @return Returns the Node, which belongs to the furthest StateTime in the GoalRegion,
   * Furthest is determined by the merit of the Node at the end of its trajectory */
  protected abstract Optional<GlcNode> getFurthestGoalNode();
}
