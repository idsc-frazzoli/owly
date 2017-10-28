// code by jph
package ch.ethz.idsc.owly.math.se2;

import java.awt.geom.AffineTransform;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.VectorQ;
import ch.ethz.idsc.tensor.mat.SquareMatrixQ;
import ch.ethz.idsc.tensor.sca.ArcTan;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public enum Se2Utils {
  ;
  // ---
  /** @param x = {px, py, angle}
   * @return matrix with dimensions 3x3
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
        { sin, cos /*----*/, x.Get(1) }, //
        { RealScalar.ZERO, RealScalar.ZERO, RealScalar.ONE }, //
    });
  }

  public static Tensor fromSE2Matrix(Tensor matrix) {
    GlobalAssert.that(SquareMatrixQ.of(matrix));
    return Tensors.of(matrix.Get(0, 2), matrix.Get(1, 2), //
        ArcTan.of(matrix.Get(0, 0), matrix.Get(1, 0)));
  }

  public static AffineTransform toAffineTransform(Tensor matrix) {
    return new AffineTransform( //
        matrix.Get(0, 0).number().doubleValue(), //
        matrix.Get(1, 0).number().doubleValue(), //
        matrix.Get(0, 1).number().doubleValue(), //
        matrix.Get(1, 1).number().doubleValue(), //
        matrix.Get(0, 2).number().doubleValue(), //
        matrix.Get(1, 2).number().doubleValue());
  }
}
