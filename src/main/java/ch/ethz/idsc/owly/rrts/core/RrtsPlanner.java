// code by jph
package ch.ethz.idsc.owly.rrts.core;

import java.util.Optional;
import java.util.PriorityQueue;

import ch.ethz.idsc.owly.data.tree.NodeCostComparator;
import ch.ethz.idsc.owly.glc.core.ExpandInterface;
import ch.ethz.idsc.owly.math.sample.RandomSample;

public class RrtsPlanner implements ExpandInterface<RrtsNode> {
  private static final RrtsNode DUMMY = new RrtsNodeImpl(null, null);
  // ---
  private final Rrts rrts;
  private final RandomSample spaceSampler;
  private final RandomSample goalSampler;
  private final PriorityQueue<RrtsNode> queue = new PriorityQueue<>(NodeCostComparator.INSTANCE);

  /** @param rrts with root already inserted
   * @param obstacleQuery
   * @param spaceSampler
   * @param goalSampler generates samples in goal region */
  public RrtsPlanner(Rrts rrts, RandomSample spaceSampler, RandomSample goalSampler) {
    this.rrts = rrts;
    this.spaceSampler = spaceSampler;
    this.goalSampler = goalSampler;
  }

  @Override
  public Optional<RrtsNode> pollNext() {
    return Optional.of(DUMMY);
  }

  @Override
  public void expand(RrtsNode node) {
    final int k_nearest = 12; // TODO magic const
    rrts.insertAsNode(spaceSampler.nextSample(), k_nearest);
    if (queue.isEmpty()) { // TODO logic not final
      Optional<RrtsNode> optional = rrts.insertAsNode(goalSampler.nextSample(), k_nearest);
      if (optional.isPresent())
        queue.add(optional.get());
    }
  }

  @Override
  public Optional<RrtsNode> getBest() {
    return Optional.ofNullable(queue.peek()); // TODO use other criterion: distance from goal center ...?
  }

  public TransitionRegionQuery getObstacleQuery() {
    return ((DefaultRrts) rrts).getObstacleQuery();
  }
}
