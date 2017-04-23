// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.tree;

import java.util.Iterator;
import java.util.PriorityQueue;
import java.util.Queue;

import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Clip;

public class Cluster<V> implements Iterable<Point<V>> {
  final Tensor center;
  private final int size;
  private final Distance distancer;
  // PriorityQueue uses canonic ordering of points: distance from center
  private final Queue<Point<V>> points = new PriorityQueue<Point<V>>();

  Cluster(Tensor center, int size, Distance distancer) {
    this.center = center;
    this.size = size;
    this.distancer = distancer;
  }

  void consider(Point<V> point) {
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
  public Iterator<Point<V>> iterator() {
    return points.iterator();
  }

  public int size() {
    return points.size();
  }
}