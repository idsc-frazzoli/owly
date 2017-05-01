// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Floor;

public abstract class TrajectoryPlanner {
  protected final Integrator integrator;
  protected final Scalar timeStep;
  private final Tensor eta;
  private final int depthLimit;
  // ---
  private final Queue<Node> queue = new PriorityQueue<>(NodeMeritComparator.instance);
  private final Map<Tensor, Node> domain_labels = new HashMap<>();

  public TrajectoryPlanner( // ..
      Integrator integrator, Scalar timeStep, Tensor eta, int depthLimit) {
    this.integrator = integrator;
    this.timeStep = timeStep;
    this.eta = eta;
    this.depthLimit = depthLimit;
  }

  public final Tensor getResolution() {
    //TODO not theoretically correct
    return eta.unmodifiable();
  }

  protected final Tensor convertToKey(Tensor x) {
    //TODO Theory: floor(eta*state) = floor(state / domain_size)
    return eta.pmul(x).map(Floor.function);
  }

  protected abstract Node createRootNode(Tensor x);

  public final void insertRoot(Tensor x) {
    insert(convertToKey(x), createRootNode(x));
  }

  protected final void insert(Tensor domain_key, Node node) {
    queue.add(node);
    domain_labels.put(domain_key, node);
  }

  protected final Node getNode(Tensor domain_key) {
    return domain_labels.get(domain_key);
  }

  private Node best;
  int replaceCount = 0; // TODO

  public final int plan(int expandLimit) {
    best = null;
    int expandCount = 0;
    while (!queue.isEmpty() && expandCount < expandLimit) {
      Node current_node = queue.poll();
      expand(current_node);
      ++expandCount;
      // ---
      if (best != null)
        break;
      if (current_node.getDepth() > depthLimit)
        break;
    }
    return expandCount;
  }

  abstract protected void expand(Node current_node);

  protected final void offerDestination(Node node) {
    if (best == null || Scalars.lessThan(node.cost, best.cost)) {
      best = node;
      System.out.println("found goal");
    }
  }

  public final Collection<Node> getQueue() {
    return Collections.unmodifiableCollection(queue);
  }

  public final Collection<Node> getNodes() {
    return Collections.unmodifiableCollection(domain_labels.values());
  }

  public final List<StateTime> getDetailedTrajectory() {
    return getDetailedTrajectory(Nodes.getNodesFromRoot(best));
  }

  public final List<StateTime> getDetailedTrajectory(List<Node> list) {
    List<StateTime> trajectory = new ArrayList<>();
    if (!list.isEmpty()) {
      trajectory.add(list.get(0).getStateTime());
      for (int index = 1; index < list.size(); ++index) {
        Node prevNode = list.get(index - 1);
        Node nextNode = list.get(index);
        final Flow flow = nextNode.flow;
        List<StateTime> part = new ArrayList<>();
        {
          StateTime prev = prevNode.getStateTime();
          while (Scalars.lessThan(prev.time, nextNode.time)) {
            Tensor x1 = integrator.step(flow, prev.x, timeStep);
            StateTime next = new StateTime(x1, prev.time.add(timeStep));
            part.add(next);
            prev = next;
          }
        }
        trajectory.addAll(part);
      }
    }
    return trajectory;
  }

  public final List<StateTime> getPathFromRootToGoal() {
    return getPathFromRootToGoal(Nodes.getNodesFromRoot(best));
  }

  public static List<StateTime> getPathFromRootToGoal(List<Node> list) {
    List<StateTime> trajectory = new ArrayList<>();
    for (Node node : list)
      trajectory.add(node.getStateTime());
    return trajectory;
  }

  public abstract TrajectoryRegionQuery getObstacleQuery();

  public abstract TrajectoryRegionQuery getGoalQuery();
}
