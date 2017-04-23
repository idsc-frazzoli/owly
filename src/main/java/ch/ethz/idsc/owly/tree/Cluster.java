// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.tree;

import java.util.HashSet;
import java.util.Iterator;
import java.util.PriorityQueue;
import java.util.Set;

import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ExtractPrimitives;

public class Cluster<V> implements Iterable<Point<V>> {
  final Tensor center;
  private final int size;
  private final Distancer distancer;
  private final PriorityQueue<Point<V>> points = new PriorityQueue<Point<V>>();
  private final double[] testPoint;

  Cluster(Tensor center, int size, Distancer distancer) {
    this.center = center;
    this.size = size;
    this.distancer = distancer;
    testPoint = new double[center.length()];
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

  void trimTo(int size) {
    while (points.size() > size)
      points.poll();
  }

  boolean isViable(double[] lBounds, double[] uBounds) {
    if (points.size() < size)
      return true;
    // ---
    for (int i = lBounds.length; --i >= 0;) {
      testPoint[i] = bound(ExtractPrimitives.toArrayDouble(center)[i], lBounds[i], uBounds[i]);
    }
    return points.peek().isFartherThan(Tensors.vectorDouble(testPoint), center, distancer);
  }

  static double bound(double value, double min, double max) {
    if (min > value)
      return min;
    if (max < value)
      return max;
    return value;
  }

  @Override
  public Iterator<Point<V>> iterator() {
    return points.iterator();
  }

  public int size() {
    return points.size();
  }

  @Override
  public boolean equals(Object object) {
    if (object instanceof Cluster) {
      @SuppressWarnings("unchecked")
      Cluster<V> cluster = (Cluster<V>) object;
      Set<Point<V>> mine = new HashSet<>(points);
      Set<Point<V>> his = new HashSet<>(cluster.points);
      return mine.equals(his);
    }
    return false;
  }

  @Override
  public String toString() {
    return points.toString();
  }
}