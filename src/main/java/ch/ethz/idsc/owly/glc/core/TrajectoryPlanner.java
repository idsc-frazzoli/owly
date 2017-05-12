// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Floor;

/** base class for generalized label correction implementation */
public abstract class TrajectoryPlanner implements ExpandInterface {
  private final Tensor eta;
  // ---
  private final Queue<Node> queue = new PriorityQueue<>(NodeMeritComparator.instance);
  // TODO long-term use RasterMap instead of domainMap
  private final Map<Tensor, Node> domainMap = new HashMap<>();
  /** best is a reference to a Node in the goal region, or null if such a node has not been identified */
  private Node best;
  private int replaceCount = 0; // see getReplaceCount()

  TrajectoryPlanner(Tensor eta) {
    this.eta = eta.copy().unmodifiable();
  }

  public final Tensor getEta() {
    return eta;
  }

  Tensor convertToKey(Tensor x) {
    // TODO Theory: floor(eta*state) = floor(state / domain_size)
    return eta.pmul(x).map(Floor.function);
  }

  abstract Node createRootNode(Tensor x);

  public final void insertRoot(Tensor x) {
    insert(convertToKey(x), createRootNode(x));
  }

  protected final void insert(Tensor domain_key, Node node) {
    queue.add(node);
    Node prev = domainMap.put(domain_key, node);
    if (prev != null)
      ++replaceCount;
  }

  protected final Node getNode(Tensor domain_key) {
    return domainMap.get(domain_key);
  }

  @Override // from ExpandInterface
  public final Node pollNext() {
    return queue.isEmpty() ? null : queue.poll();
  }

  /** peek does not change the queue state
   * 
   * @return next node in the queue */
  public final Node peek() {
    return queue.peek();
  }

  final void offerDestination(Node node) {
    if (best == null || Scalars.lessThan(node.cost(), best.cost())) {
      best = node;
      System.out.println("found goal");
    }
  }

  @Override // from ExpandInterface
  public final Node getBest() {
    return best;
  }

  /** @return number of replacements in the domain map caused by {@link TrajectoryPlanner#insert(Tensor, Node)} */
  public final int getReplaceCount() {
    return replaceCount;
  }

  /** @param node
   * @return densely sampled trajectory from root to given node
   * that is the result of integrating the flows between the nodes */
  public abstract List<StateTime> detailedTrajectoryTo(Node node);

  /** @return obstacle query for the purpose of inspection, i.e. no alteration should be made */
  public abstract TrajectoryRegionQuery getObstacleQuery();

  /** @return goal query for the purpose of inspection, i.e. no alteration should be made */
  public abstract TrajectoryRegionQuery getGoalQuery();

  protected final Collection<Node> queue() { 
    return queue;
  }
  protected final Map<Tensor, Node> domainMap() {
    return domainMap;
  }

  public final Collection<Node> getQueue() {
    return Collections.unmodifiableCollection(queue);
  }

  public final Collection<Node> getNodes() {
    return Collections.unmodifiableCollection(domainMap.values());
  }

  // TODO rename to coarse path ...
  public final List<StateTime> getPathFromRootToGoal() {
    return Nodes.nodesFromRoot(best).stream().map(Node::stateTime).collect(Collectors.toList());
  }
}
