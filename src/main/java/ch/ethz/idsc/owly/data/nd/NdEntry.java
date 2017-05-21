// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.data.nd;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class NdEntry<V> implements Comparable<NdEntry<V>>, Serializable {
  public final Tensor location; // <- key
  public final V value;
  public Scalar distanceToCenter;

  /* package */ NdEntry(Tensor location, V value) {
    this.location = location;
    this.value = value;
  }

  /* package */ void setDistanceToCenter(NdDistanceInterface distancer, Tensor center) {
    distanceToCenter = distancer.apply(center, location);
  }

  /* package */ boolean isFartherThan(Tensor testPoint, Tensor center, NdDistanceInterface distancer) {
    return Scalars.lessThan(distancer.apply(testPoint, center), distanceToCenter);
  }

  @Override
  public int compareTo(NdEntry<V> other) {
    return Scalars.compare(other.distanceToCenter, distanceToCenter);
  }

  @Override
  public int hashCode() {
    return location.hashCode() ^ value.hashCode();
  }

  @Override
  public boolean equals(Object object) {
    if (object instanceof NdEntry) {
      @SuppressWarnings("unchecked")
      NdEntry<V> other = (NdEntry<V>) object;
      return location.equals(other.location) && value.equals(other.value);
    }
    return false;
  }
}