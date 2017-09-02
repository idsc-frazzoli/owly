// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.data.nd;

import java.io.Serializable;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.stream.Stream;

import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Clip;

public class NdCluster<V> implements Iterable<NdEntry<V>>, Serializable {
  private final NdCenterInterface ndCenter;
  /* package */ final Tensor center;
  private final int limit;
  private final Queue<NdEntry<V>> queue;
  private int considered = 0;

  /* package */ NdCluster(Collection<NdPair<V>> collection, NdCenterInterface ndCenter, int limit) {
    this.ndCenter = ndCenter;
    this.center = ndCenter.center();
    this.limit = limit;
    queue = new LinkedList<>();
    collection.stream() //
        .map(pair -> new NdEntry<>(pair, ndCenter.ofVector(pair.location))) //
        .sorted(NdEntryComparators.INCREASING) //
        .limit(limit) //
        .forEach(queue::add);
  }

  /* package */ NdCluster(NdCenterInterface ndCenter, int limit) {
    this.ndCenter = ndCenter;
    this.center = ndCenter.center();
    this.limit = limit;
    queue = new PriorityQueue<NdEntry<V>>(NdEntryComparators.DECREASING);
  }

  /* package */ void consider(NdPair<V> pair) {
    ++considered;
    NdEntry<V> point = new NdEntry<>(pair, ndCenter.ofVector(pair.location));
    if (queue.size() < limit)
      queue.add(point);
    else //
    if (Scalars.lessThan(point.distance, queue.peek().distance)) {
      queue.poll();
      queue.add(point);
    }
  }

  /* package */ boolean isViable(Tensor lBounds, Tensor uBounds) {
    if (queue.size() < limit)
      return true;
    // ---
    Tensor test = Tensors.vector( //
        i -> Clip.function(lBounds.Get(i), uBounds.Get(i)).apply(center.Get(i)), center.length());
    return Scalars.lessThan(ndCenter.ofVector(test), queue.peek().distance);
  }

  /** @return number of points visited in order to build the cluster */
  public int considered() {
    return considered;
  }

  @Override
  public Iterator<NdEntry<V>> iterator() {
    return queue.iterator();
  }

  public Stream<NdEntry<V>> stream() {
    return queue.stream();
  }

  public Collection<NdEntry<V>> collection() {
    return Collections.unmodifiableCollection(queue);
  }

  public int size() {
    return queue.size();
  }
}