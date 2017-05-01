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
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Floor;

public abstract class TrajectoryPlanner implements ExpandInterface {
  protected final IntegrationConfig integrationConfig;
  private final Tensor partitionScale;
  // ---
  private final Queue<Node> queue = new PriorityQueue<>(NodeMeritComparator.instance);
  private final Map<Tensor, Node> domain_labels = new HashMap<>();

  public TrajectoryPlanner( // ..
      IntegrationConfig integrationConfig, Tensor partitionScale) {
    this.integrationConfig = integrationConfig;
    this.partitionScale = partitionScale;
  }

  public final Tensor getResolution() {
    return partitionScale.unmodifiable();
  }

  protected final Tensor convertToKey(Tensor x) {
    return partitionScale.pmul(x).map(Floor.function);
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

  @Override
  public final Node pollNext() {
    return queue.isEmpty() ? null : queue.poll();
  }

  protected final void offerDestination(Node node) {
    if (best == null || Scalars.lessThan(node.cost, best.cost)) {
      best = node;
      System.out.println("found goal");
    }
  }

  @Override
  public final Node getBest() {
    return best;
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
          // while (Scalars.lessThan(prev.time, nextNode.time)) {
          for (int c0 = 0; c0 < integrationConfig.trajectorySize; ++c0) {
            Tensor x1 = integrationConfig.integrator.step(flow, prev.x, integrationConfig.timeStep);
            StateTime next = new StateTime(x1, prev.time.add(integrationConfig.timeStep));
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
