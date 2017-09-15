// code by jph
package ch.ethz.idsc.owly.demo.drift;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public enum DriftStates {
  ;
  // used in owly3d
  public static Tensor x0_demo1() {
    return Tensors.vector(0, 0, 0, 0, 0, 1);
  }
}
