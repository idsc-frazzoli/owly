// code by jph
package ch.ethz.idsc.owly.demo.glc.se2;

import ch.ethz.idsc.tensor.ComplexScalar;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Arg;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public enum Se2Utils {
  ;
  public static final Scalar DEGREE(int value) {
    return RationalScalar.of(value, 180).multiply(DoubleScalar.of(Math.PI));
  }

  // ---
  // function not called
  static Tensor vec2mat(Tensor x) {
    Scalar angle = x.Get(2);
    return Tensors.matrix(new Tensor[][] { //
        { Cos.of(angle), Sin.of(angle).negate(), x.Get(0) }, //
        { Sin.of(angle), Cos.of(angle), x.Get(1) }, //
        { RealScalar.ZERO, RealScalar.ZERO, RealScalar.ONE }, //
    });
  }

  // function not called
  static Tensor mat2vec(Tensor mat) {
    Scalar arg = Arg.of(ComplexScalar.of(mat.Get(0, 0), mat.Get(1, 0)));
    return Tensors.of(mat.get(0, 2), mat.get(1, 2), arg);
  }
}
