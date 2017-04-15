// code by jph
package ch.ethz.idsc.owly.demo.glc.psu;

import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Partition;
import ch.ethz.idsc.tensor.alg.Subdivide;

class PsuControls {
  public static Tensor createControls(int num) {
    return Partition.of(Subdivide.of(DoubleScalar.of(-0.2), DoubleScalar.of(0.2), num), 1);
  }
}
