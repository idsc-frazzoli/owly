// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.tree;

import java.util.LinkedList;
import java.util.Queue;

import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class NdTreeMap<V> {
  private static final Scalar HALF = RationalScalar.of(1, 2);
  private final Node root;
  private final int maxDensity;
  private int size;
  private int numRemoved;
  private final Tensor global_lBounds;
  private final Tensor global_uBounds;
  // ---
  // reused during adding as well as searching:
  private Tensor lBounds;
  private Tensor uBounds;

  public NdTreeMap(Tensor lbounds, Tensor ubounds, int maxDensity, int maxDepth) {
    global_lBounds = lbounds;
    global_uBounds = ubounds;
    this.maxDensity = maxDensity;
    root = new Node(maxDepth);
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

  public Cluster<V> buildCluster(Tensor center, int size, Distance distancer) {
    resetBounds();
    Cluster<V> cluster = new Cluster<V>(center, size, distancer);
    root.addToCluster(cluster);
    return cluster;
  }

  public int size() {
    return size;
  }

  private void resetBounds() {
    lBounds = global_lBounds.copy();
    uBounds = global_uBounds.copy();
  }

  private class Node {
    private final int depth;
    private boolean internal = false;
    private Node lChild;
    private Node rChild;
    private Queue<Point<V>> queue = new LinkedList<Point<V>>();

    Node(int depth) {
      this.depth = depth;
    }

    private Scalar median(int index) {
      return lBounds.Get(index).add(uBounds.Get(index)).multiply(HALF);
    }

    Point<V> add(final Point<V> point) {
      if (internal) {
        Tensor location = point.location;
        int dimension = depth % location.length();
        Scalar median = median(dimension);
        if (Scalars.lessThan(location.Get(dimension), median)) {
          uBounds.set(median, dimension);
          if (lChild == null)
            lChild = new Node(depth - 1);
          return lChild.add(point);
        }
        lBounds.set(median, dimension);
        if (rChild == null)
          rChild = new Node(depth - 1);
        return rChild.add(point);
      }
      if (queue.size() < maxDensity) {
        queue.add(point);
        return null;
      }
      if (depth == 1) {
        queue.add(point);
        return queue.poll();
      }
      int dimension = depth % lBounds.length();
      Scalar median = median(dimension);
      for (Point<V> p : queue)
        if (Scalars.lessThan(p.location.Get(dimension), median)) {
          if (lChild == null)
            lChild = new Node(depth - 1);
          lChild.queue.add(p);
        } else {
          if (rChild == null)
            rChild = new Node(depth - 1);
          rChild.queue.add(p);
        }
      queue = null;
      internal = true;
      return add(point);
    }

    void addToCluster(Cluster<V> cluster) {
      if (internal) {
        final int dimension = depth % lBounds.length();
        Scalar median = median(dimension);
        boolean lFirst = Scalars.lessThan(cluster.center.Get(dimension), median);
        addChildToCluster(cluster, median, lFirst);
        addChildToCluster(cluster, median, !lFirst);
      } else {
        for (Point<V> point : queue)
          cluster.consider(point);
      }
    }

    private void addChildToCluster(Cluster<V> cluster, Scalar median, boolean left) {
      int dimension = depth % lBounds.length();
      if (left) {
        if (lChild == null)
          return;
        // ---
        Scalar copy = uBounds.Get(dimension);
        uBounds.set(median, dimension);
        if (cluster.isViable(lBounds, uBounds))
          lChild.addToCluster(cluster);
        uBounds.set(copy, dimension);
      } else {
        if (rChild == null)
          return;
        // ---
        Scalar copy = lBounds.Get(dimension);
        lBounds.set(median, dimension);
        if (cluster.isViable(lBounds, uBounds))
          rChild.addToCluster(cluster);
        lBounds.set(copy, dimension);
      }
    }
  }
}
