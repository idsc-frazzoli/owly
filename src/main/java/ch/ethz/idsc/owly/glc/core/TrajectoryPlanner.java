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
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.sca.Floor;

public abstract class TrajectoryPlanner {
  protected final Integrator integrator;
  protected final DynamicalSystem dynamicalSystem;
  protected final Controls controls;
  protected final CostFunction costFunction;
  protected final Heuristic heuristic;
  protected final TrajectoryRegionQuery goalQuery;
  protected final TrajectoryRegionQuery obstacleQuery;
  // ---
  protected Tensor partitionScale;
  protected Scalar expand_time = RealScalar.ONE; // TODO
  protected final Queue<Node> queue = new PriorityQueue<>(NodeMeritComparator.instance);
  protected Map<Tensor, Node> domain_labels = new HashMap<>();

  public TrajectoryPlanner( //
      Integrator integrator, //
      DynamicalSystem dynamicalSystem, //
      Controls controls, //
      CostFunction costFunction, //
      Heuristic heuristic, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    this.integrator = integrator;
    this.dynamicalSystem = dynamicalSystem;
    this.controls = controls;
    this.costFunction = costFunction;
    this.heuristic = heuristic;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
  }

  public void setResolution(Tensor partitionScale) {
    this.partitionScale = partitionScale;
  }

  public Tensor getResolution() {
    return partitionScale;
  }

  private Tensor convertToKey(Tensor x) {
    return partitionScale.pmul(x).map(Floor.function);
  }

  public void insertRoot(Tensor x) {
    insert(convertToKey(x), //
        new Node(null, x, ZeroScalar.get(), ZeroScalar.get(), ZeroScalar.get()));
  }

  private void insert(Tensor domain_key, Node node) {
    queue.add(node);
    domain_labels.put(domain_key, node);
  }

  int depth_limit;
  Node best;
  int replaceCount = 0; // TODO

  public final void plan(int depth_limit) {
    this.depth_limit = depth_limit;
    best = null;
    while (expand()) {
      // ---
    }
  }

  abstract boolean expand();

  public Collection<Node> getQueue() {
    return Collections.unmodifiableCollection(queue);
  }

  public Collection<Node> getNodes() {
    return Collections.unmodifiableCollection(domain_labels.values());
  }

  public Trajectory getDetailedTrajectory() {
    return getDetailedTrajectory(Nodes.getNodesFromRoot(best));
  }

  public Trajectory getDetailedTrajectory(List<Node> list) {
    Trajectory trajectory = new Trajectory();
    for (int index = 1; index < list.size(); ++index) {
      Node prev = list.get(index - 1);
      Node next = list.get(index);
      Trajectory part = dynamicalSystem.sim(integrator, next.u, prev.time, prev.time.add(expand_time), prev.x);
      trajectory.addAll(part);
    }
    return trajectory;
  }

  public Trajectory getPathFromRootToGoal() {
    return getPathFromRootToGoal(Nodes.getNodesFromRoot(best));
  }

  public Trajectory getPathFromRootToGoal(List<Node> list) {
    Trajectory trajectory = new Trajectory();
    for (Node node : list)
      trajectory.add(node.getStateTime());
    return trajectory;
  }
}
