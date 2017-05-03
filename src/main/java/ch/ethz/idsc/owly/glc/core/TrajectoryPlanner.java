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
  /** number of replacements in the domain map */
  private int replaceCount = 0;

  TrajectoryPlanner(Tensor eta) {
    this.eta = eta;
  }

  public final Tensor getResolution() {
    // TODO not theoretically correct
    return eta.unmodifiable();
  }

  protected Tensor convertToKey(Tensor x) {
    // TODO Theory: floor(eta*state) = floor(state / domain_size)
    return eta.pmul(x).map(Floor.function);
  }

  protected abstract Node createRootNode(Tensor x);

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

  protected final void offerDestination(Node node) {
    if (best == null || Scalars.lessThan(node.cost(), best.cost())) {
      best = node;
      System.out.println("found goal");
    }
  }

  @Override // from ExpandInterface
  public final Node getBest() {
    return best;
  }

  public int getReplaceCount() {
    return replaceCount;
  }

  public abstract List<StateTime> detailedTrajectoryTo(Node node);

  public final Collection<Node> getQueue() {
    return Collections.unmodifiableCollection(queue);
  }

  public final Collection<Node> getNodes() {
    return Collections.unmodifiableCollection(domainMap.values());
  }

  public final List<StateTime> getPathFromRootToGoal() {
    return getPathFromRootToGoal(Nodes.nodesFromRoot(best));
  }

  // TODO function is only used once...
  private static List<StateTime> getPathFromRootToGoal(List<Node> list) {
    return list.stream().map(Node::stateTime).collect(Collectors.toList());
  }

  public abstract TrajectoryRegionQuery getObstacleQuery();

  public abstract TrajectoryRegionQuery getGoalQuery();
}
