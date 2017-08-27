// code by jph
package ch.ethz.idsc.owly.rrts.core;

import java.util.Optional;
import java.util.PriorityQueue;

import ch.ethz.idsc.owly.data.tree.NodeCostComparator;

public class RrtsPlanner implements ExploreInterface {
  private final Rrts rrts;
  private final RandomSampleInterface spaceSampler;
  private final RandomSampleInterface goalSampler;
  // private final Collection<RrtsNode> collection = new LinkedList<>();
  private final PriorityQueue<RrtsNode> queue = new PriorityQueue<>(NodeCostComparator.INSTANCE);

  /** @param rrts with root already inserted
   * @param obstacleQuery
   * @param spaceSampler
   * @param goalSampler generates samples in goal region */
  public RrtsPlanner(Rrts rrts, RandomSampleInterface spaceSampler, RandomSampleInterface goalSampler) {
    this.rrts = rrts;
    this.spaceSampler = spaceSampler;
    this.goalSampler = goalSampler;
  }

  @Override
  public Optional<RrtsNode> next() {
    final int k_nearest = 12; // TODO magic const
    rrts.insertAsNode(spaceSampler.nextSample(), k_nearest);
    Optional<RrtsNode> optional = rrts.insertAsNode(goalSampler.nextSample(), k_nearest);
    if (optional.isPresent())
      queue.add(optional.get());
    return optional;
  }

  @Override
  public Optional<RrtsNode> getBest() {
    return Optional.ofNullable(queue.peek());
  }
}
