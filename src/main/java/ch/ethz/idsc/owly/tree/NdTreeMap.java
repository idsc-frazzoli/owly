// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.tree;

import java.util.LinkedList;
import java.util.Queue;
import java.util.stream.IntStream;

import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class NdTreeMap<V> {
  private final Node root;
  private final int maxDensity;
  private int size;
  private int numRemoved;
  private final Tensor global_lBounds;
  private final Tensor global_uBounds;
  private final double[] lBounds;
  private final double[] uBounds;

  public NdTreeMap(Tensor lbounds, Tensor ubounds, int maxDensity, int maxDepth) {
    global_lBounds = lbounds;
    global_uBounds = ubounds;
    this.maxDensity = maxDensity;
    root = new Node(maxDepth);
    lBounds = new double[global_lBounds.length()];
    uBounds = new double[global_uBounds.length()];
  }

  public void add(Tensor location, V value) {
    add(new Point<V>(location, value));
  }

  private Point<V> add(Point<V> point) {
    resetBounds();
    Point<V> removed = root.add(point);
    if (removed == null) {
      ++size;
    } else {
      ++numRemoved;
      // System.out.printf("Removed %s (#%d)\n", Arrays
      // .toString(removed.location), ++numRemoved);
    }
    return removed;
  }

  public Cluster<V> buildCluster(Tensor center, int size, Distancer distancer) {
    resetBounds();
    Cluster<V> cluster = new Cluster<V>(center, size, distancer);
    root.addToCluster(cluster);
    return cluster;
  }

  public int size() {
    return size;
  }

  private void resetBounds() {
    IntStream.range(0, lBounds.length).boxed().forEach(index -> {
      lBounds[index] = global_lBounds.Get(index).number().doubleValue();
      uBounds[index] = global_uBounds.Get(index).number().doubleValue();
    });
  }

  private class Node {
    private final int depth;
    private boolean internal = false;
    private Node lChild;
    private Node rChild;
    private Queue<Point<V>> data = new LinkedList<Point<V>>();

    Node(int depth) {
      this.depth = depth;
    }

    private double median(int dimension) {
      return (lBounds[dimension] + uBounds[dimension]) * 0.5;
    }

    Point<V> add(final Point<V> point) {
      if (internal) {
        Tensor location = point.location;
        int dimension = depth % location.length();
        double median = median(dimension);
        if (Scalars.lessThan(location.Get(dimension), DoubleScalar.of(median))) {
          uBounds[dimension] = median;
          if (lChild == null)
            lChild = new Node(depth - 1);
          return lChild.add(point);
        }
        lBounds[dimension] = median;
        if (rChild == null)
          rChild = new Node(depth - 1);
        return rChild.add(point);
      }
      if (data.size() < maxDensity) {
        data.add(point);
        return null;
      }
      if (depth == 1) {
        data.add(point);
        return data.poll();
      }
      int dimension = depth % lBounds.length;
      double median = median(dimension);
      for (Point<V> p : data)
        if (Scalars.lessThan(p.location.Get(dimension), DoubleScalar.of(median))) {
          if (lChild == null)
            lChild = new Node(depth - 1);
          lChild.data.add(p);
        } else {
          if (rChild == null)
            rChild = new Node(depth - 1);
          rChild.data.add(p);
        }
      data = null;
      internal = true;
      return add(point);
    }

    void addToCluster(Cluster<V> cluster) {
      if (internal) {
        final int dimension = depth % lBounds.length;
        double median = median(dimension);
        boolean lFirst = Scalars.lessThan(cluster.center.Get(dimension), DoubleScalar.of(median));
        addChildToCluster(cluster, median, lFirst);
        addChildToCluster(cluster, median, !lFirst);
      } else {
        for (Point<V> point : data)
          cluster.consider(point);
      }
    }

    private void addChildToCluster(Cluster<V> cluster, double median, boolean left) {
      int dimension = depth % lBounds.length;
      if (left) {
        if (lChild == null)
          return;
        // ---
        double orig = uBounds[dimension];
        uBounds[dimension] = median;
        if (cluster.isViable(lBounds, uBounds))
          lChild.addToCluster(cluster);
        uBounds[dimension] = orig;
      } else {
        if (rChild == null)
          return;
        // ---
        double orig = lBounds[dimension];
        lBounds[dimension] = median;
        if (cluster.isViable(lBounds, uBounds))
          rChild.addToCluster(cluster);
        lBounds[dimension] = orig;
      }
    }
  }
}
