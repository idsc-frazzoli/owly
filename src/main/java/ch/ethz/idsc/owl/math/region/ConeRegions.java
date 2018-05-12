// code by jph
package ch.ethz.idsc.owl.math.region;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public enum ConeRegions {
  ;
  public static Region<Tensor> planar(Tensor xya, Scalar semi) {
    return new Cone2Region(xya, semi);
  }
}
