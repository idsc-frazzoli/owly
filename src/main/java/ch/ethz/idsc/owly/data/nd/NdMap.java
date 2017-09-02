// code by jph
package ch.ethz.idsc.owly.data.nd;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Tensor;

/** multiple values can be associated to the same key
 * 
 * @param <V> */
public interface NdMap<V> extends Serializable {
  /** @param location
   * @param value */
  void add(Tensor location, V value);

  /** @return number of entries stored in map */
  int size();

  NdCluster<V> buildCluster(Tensor center, int size, NdDistanceInterface distancer);
}
