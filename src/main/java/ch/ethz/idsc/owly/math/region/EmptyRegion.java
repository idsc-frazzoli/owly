// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.Tensor;

/** empty region.
 * no given tensor is a member */
public enum EmptyRegion implements Region<Tensor> {
  INSTANCE;
  // ---
  // TODO
  // public static <T> Region<T> EMPTY() {
  // return new Region<T>() {
  // @Override
  // public boolean isMember(T type) {
  // Collections.emptyList();
  // return false;
  // };
  // };
  // }
  @Override
  public boolean isMember(Tensor tensor) {
    return false;
  }
}
