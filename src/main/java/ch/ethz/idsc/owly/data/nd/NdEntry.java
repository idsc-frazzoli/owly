// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.data.nd;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class NdEntry<V> implements Serializable {
  private final NdPair<V> pair;
  public final Scalar distanceToCenter;

  NdEntry(NdPair<V> ndPair, Scalar distance) {
    pair = ndPair;
    distanceToCenter = distance;
  }

  /* package */ boolean isFartherThan(Tensor testPoint, Tensor center, NdDistanceInterface distancer) {
    return Scalars.lessThan(distancer.apply(testPoint, center), distanceToCenter);
  }

  public Tensor location() {
    return pair.location;
  }

  public V value() {
    return pair.value();
  }
}