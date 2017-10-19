// code by jph
package ch.ethz.idsc.owly.math;

import java.awt.geom.AffineTransform;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.VectorQ;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public enum Se2Utils {
  ;
  // ---
  /** @param x = {px, py, angle}
   * @return 3x3 matrix
   * [+Ca -Sa px]
   * [+Sa +Ca py]
   * [0 0 1] */
  public static Tensor toSE2Matrix(Tensor x) {
    GlobalAssert.that(VectorQ.ofLength(x, 3));
    Scalar angle = x.Get(2);
    Scalar cos = Cos.of(angle);
    Scalar sin = Sin.of(angle);
    return Tensors.matrix(new Tensor[][] { //
        { cos, sin.negate(), x.Get(0) }, //
        { sin, cos, x.Get(1) }, //
        { RealScalar.ZERO, RealScalar.ZERO, RealScalar.ONE }, //
    });
  }

  public static AffineTransform toAffineTransform(Tensor m) {
    return new AffineTransform( //
        m.Get(0, 0).number().doubleValue(), //
        m.Get(1, 0).number().doubleValue(), //
        m.Get(0, 1).number().doubleValue(), //
        m.Get(1, 1).number().doubleValue(), //
        m.Get(0, 2).number().doubleValue(), //
        m.Get(1, 2).number().doubleValue());
  }
}
