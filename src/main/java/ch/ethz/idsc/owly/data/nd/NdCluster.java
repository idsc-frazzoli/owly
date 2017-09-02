// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.data.nd;

import java.io.Serializable;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.stream.Stream;

import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Clip;

// TODO create NdClusterBuilder!
public class NdCluster<V> implements Iterable<NdEntry<V>>, Serializable {
  final Tensor center;
  private final int size;
  private NdDistanceInterface distancer;
  private final Queue<NdEntry<V>> points;

  NdCluster(Tensor center, Stream<NdEntry<V>> stream) {
    this.center = center;
    points = new LinkedList<>();
    stream.forEach(points::add);
    size = points.size();
  }

  /* package */ NdCluster(Tensor center, int size, NdDistanceInterface distancer) {
    this.center = center;
    this.size = size;
    this.distancer = distancer;
    points = new PriorityQueue<NdEntry<V>>(NdEntryComparators.DECREASING);
  }

  /* package */ void consider(NdPair<V> pair) {
    NdEntry<V> point = new NdEntry<>(pair, distancer.apply(center, pair.location));
    if (points.size() < size)
      points.add(point);
    else //
    if (Scalars.lessThan(point.distanceToCenter, points.peek().distanceToCenter)) {
      points.poll();
      points.add(point);
    }
  }

  /* package */ boolean isViable(Tensor lBounds, Tensor uBounds) {
    if (points.size() < size)
      return true;
    // ---
    Tensor test = Tensors.vector( //
        i -> Clip.function(lBounds.Get(i), uBounds.Get(i)).apply(center.Get(i)), center.length());
    return points.peek().isFartherThan(test, center, distancer);
  }

  @Override
  public Iterator<NdEntry<V>> iterator() {
    return points.iterator();
  }

  public Stream<NdEntry<V>> stream() {
    return points.stream();
  }

  public int size() {
    return points.size();
  }
}