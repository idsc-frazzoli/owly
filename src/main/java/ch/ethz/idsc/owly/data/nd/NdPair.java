// code by jph
package ch.ethz.idsc.owly.data.nd;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Tensor;

public class NdPair<V> implements Serializable {
  public final Tensor location; // <- key
  public final V value;

  /* package */ NdPair(Tensor location, V value) {
    this.location = location.unmodifiable();
    this.value = value;
  }

  public V value() {
    return value;
  }
}