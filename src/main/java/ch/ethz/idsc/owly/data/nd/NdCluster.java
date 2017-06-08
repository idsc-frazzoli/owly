// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.data.nd;

import java.util.Iterator;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.stream.Stream;

import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Clip;

public class NdCluster<V> implements Iterable<NdEntry<V>> {
  final Tensor center;
  private final int size;
  private final NdDistanceInterface distancer;
  // PriorityQueue uses canonic ordering of points: distance from center
  private final Queue<NdEntry<V>> points = new PriorityQueue<NdEntry<V>>();

  NdCluster(Tensor center, int size, NdDistanceInterface distancer) {
    this.center = center;
    this.size = size;
    this.distancer = distancer;
  }

  void consider(NdEntry<V> point) {
    point.setDistanceToCenter(distancer, center);
    if (points.size() < size) {
      points.add(point);
    } else {
      if (Scalars.lessThan(point.distanceToCenter, points.peek().distanceToCenter)) {
        points.poll();
        points.add(point);
      }
    }
  }

  @Deprecated
  void trimTo(int size) {
    while (points.size() > size)
      points.poll();
  }

  boolean isViable(Tensor lBounds, Tensor uBounds) {
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