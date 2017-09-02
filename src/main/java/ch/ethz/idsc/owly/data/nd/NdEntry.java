// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.data.nd;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class NdEntry<V> implements Serializable {
  public final Tensor location; // <- key
  public final V value;
  public Scalar distanceToCenter; // bad style!!!

  NdEntry(NdPair<V> ndPair, Scalar distance) {
    location = ndPair.location;
    value = ndPair.value;
    distanceToCenter = distance;
  }

  /* package */ NdEntry(Tensor location, V value) {
    this.location = location.unmodifiable();
    this.value = value;
  }

  /* package */ void setDistanceToCenter(NdDistanceInterface distancer, Tensor center) {
    distanceToCenter = distancer.apply(center, location);
  }

  /* package */ boolean isFartherThan(Tensor testPoint, Tensor center, NdDistanceInterface distancer) {
    return Scalars.lessThan(distancer.apply(testPoint, center), distanceToCenter);
  }

  public V value() {
    return value;
  }
}