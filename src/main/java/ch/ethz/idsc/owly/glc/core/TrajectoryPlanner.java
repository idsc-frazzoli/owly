// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;

import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.sca.Floor;

public abstract class TrajectoryPlanner {
  protected final Integrator integrator;
  protected final Scalar maxTimeStep;
  // ---
  private Tensor partitionScale;
  private final Queue<Node> queue = new PriorityQueue<>(NodeMeritComparator.instance);
  private final Map<Tensor, Node> domain_labels = new HashMap<>();

  public TrajectoryPlanner(Integrator integrator, Scalar maxTimeStep) {
    this.integrator = integrator;
    this.maxTimeStep = maxTimeStep;
  }

  public final void setResolution(Tensor partitionScale) {
    this.partitionScale = partitionScale;
  }

  public final Tensor getResolution() {
    return partitionScale;
  }

  protected final Tensor convertToKey(Tensor x) {
    return partitionScale.pmul(x).map(Floor.function);
  }

  public final void insertRoot(Tensor x) {
    insert(convertToKey(x), //
        new Node(null, x, ZeroScalar.get(), ZeroScalar.get(), ZeroScalar.get()));
  }

  protected final void insert(Tensor domain_key, Node node) {
    queue.add(node);
    domain_labels.put(domain_key, node);
  }

  protected final Node getNode(Tensor domain_key) {
    return domain_labels.get(domain_key);
  }

  // protected final Trajectory evolve(Flow flow, Node node) {
  // return dynamicalSystem.sim(integrator, flow, node.time, node.time.add(expand_time), node.x);
  // }
  private Node best;
  int replaceCount = 0; // TODO

  public final void plan(int depth_limit) {
    best = null;
    while (!queue.isEmpty()) {
      Node current_node = queue.poll();
      // System.out.println(current_node);
      if (depth_limit < current_node.depth) {
        System.out.println("depth limit reached " + current_node.depth);
        break;
      }
      expand(current_node);
      // ---
      if (best != null)
        break;
    }
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

  public final Trajectory getDetailedTrajectory() {
    return getDetailedTrajectory(Nodes.getNodesFromRoot(best));
  }

  public final Trajectory getDetailedTrajectory(List<Node> list) {
    Trajectory trajectory = new Trajectory();
    for (int index = 1; index < list.size(); ++index) {
      Node prev = list.get(index - 1);
      Node next = list.get(index);
      Trajectory part = new Trajectory(); // .sim(integrator, next.u, prev.time, prev.time.add(expand_time), prev.x);
      trajectory.addAll(part);
    }
    return trajectory;
  }

  public final Trajectory getPathFromRootToGoal() {
    return getPathFromRootToGoal(Nodes.getNodesFromRoot(best));
  }

  public static Trajectory getPathFromRootToGoal(List<Node> list) {
    Trajectory trajectory = new Trajectory();
    for (Node node : list)
      trajectory.add(node.getStateTime());
    return trajectory;
  }
}
