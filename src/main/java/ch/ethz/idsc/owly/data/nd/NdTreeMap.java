// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.data.nd;

import java.io.Serializable;
import java.util.LinkedList;
import java.util.Objects;
import java.util.Queue;

import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class NdTreeMap<V> implements Serializable {
  private static final Scalar HALF = RationalScalar.of(1, 2);
  // ---
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
    add(new NdEntry<V>(location, value));
  }

  private NdEntry<V> add(NdEntry<V> point) {
    resetBounds();
    NdEntry<V> removed = root.add(point);
    if (Objects.isNull(removed)) {
      ++size;
    } else {
      ++numRemoved;
      throw new RuntimeException();
      // System.out.printf("Removed %s (#%d)\n", Arrays
      // .toString(removed.location), ++numRemoved);
    }
    return removed;
  }

  public NdCluster<V> buildCluster(Tensor center, int size, NdDistanceInterface distancer) {
    resetBounds();
    NdCluster<V> cluster = new NdCluster<V>(center, size, distancer);
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

  private class Node implements Serializable {
    private final int depth;
    private boolean internal = false;
    private Node lChild;
    private Node rChild;
    private Queue<NdEntry<V>> queue = new LinkedList<NdEntry<V>>();

    Node(int depth) {
      this.depth = depth;
    }

    private Scalar median(int index) {
      return lBounds.Get(index).add(uBounds.Get(index)).multiply(HALF);
    }

    NdEntry<V> add(final NdEntry<V> point) {
      if (internal) {
        Tensor location = point.location;
        int dimension = depth % location.length();
        Scalar median = median(dimension);
        if (Scalars.lessThan(location.Get(dimension), median)) {
          uBounds.set(median, dimension);
          if (Objects.isNull(lChild))
            lChild = new Node(depth - 1);
          return lChild.add(point);
        }
        lBounds.set(median, dimension);
        if (Objects.isNull(rChild))
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
      for (NdEntry<V> p : queue)
        if (Scalars.lessThan(p.location.Get(dimension), median)) {
          if (Objects.isNull(lChild))
            lChild = new Node(depth - 1);
          lChild.queue.add(p);
        } else {
          if (Objects.isNull(rChild))
            rChild = new Node(depth - 1);
          rChild.queue.add(p);
        }
      queue = null;
      internal = true;
      return add(point);
    }

    void addToCluster(NdCluster<V> cluster) {
      if (internal) {
        final int dimension = depth % lBounds.length();
        Scalar median = median(dimension);
        boolean lFirst = Scalars.lessThan(cluster.center.Get(dimension), median);
        addChildToCluster(cluster, median, lFirst);
        addChildToCluster(cluster, median, !lFirst);
      } else {
        for (NdEntry<V> point : queue)
          cluster.consider(point);
      }
    }

    private void addChildToCluster(NdCluster<V> cluster, Scalar median, boolean left) {
      int dimension = depth % lBounds.length();
      if (left) {
        if (Objects.isNull(lChild))
          return;
        // ---
        Scalar copy = uBounds.Get(dimension);
        uBounds.set(median, dimension);
        if (cluster.isViable(lBounds, uBounds))
          lChild.addToCluster(cluster);
        uBounds.set(copy, dimension);
      } else {
        if (Objects.isNull(rChild))
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
