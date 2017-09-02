// code by jph
package ch.ethz.idsc.owly.data.nd;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Tensor;

/** NdMap contains (coordinate, value)-pairs.
 * multiple values can be associated to the same coordinate.
 * 
 * @param <V> */
public interface NdMap<V> extends Serializable {
  /** @param location
   * @param value */
  void add(Tensor location, V value);

  /** @return number of entries stored in map */
  int size();

  /** @param distancer
   * @param limit
   * @return */
  NdCluster<V> buildCluster(NdCenterInterface distancer, int limit);
}
