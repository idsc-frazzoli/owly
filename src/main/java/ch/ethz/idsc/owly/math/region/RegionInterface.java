// code by jph
package ch.ethz.idsc.owly.math.region;

import java.io.Serializable;

/** determines membership for elements of type T */
public interface RegionInterface<T> extends Serializable {
  /** @param type
   * @return membership status of given type */
  boolean isMember(T type);
}
