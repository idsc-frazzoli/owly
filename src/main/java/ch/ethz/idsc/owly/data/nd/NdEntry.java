// code by Eric Simonton
// adapted by jph and clruch
package ch.ethz.idsc.owly.data.nd;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class NdEntry<V> implements Serializable {
  private final NdPair<V> ndPair;
  public final Scalar distance;

  /* package */ NdEntry(NdPair<V> ndPair, Scalar distance) {
    this.ndPair = ndPair;
    this.distance = distance;
  }

  public Tensor location() {
    return ndPair.location;
  }

  public V value() {
    return ndPair.value();
  }
}