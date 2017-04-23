// code by jph
package ch.ethz.idsc.owly.demo.glc.se2;

import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.tensor.ComplexScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.sca.ArgInterface;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

enum Se2Utils {
  ;
  // ---
  public static Tensor vec2mat(Tensor x) {
    Scalar angle = x.Get(2);
    return Tensors.matrix(new Tensor[][] { //
        { Cos.of(angle), Sin.of(angle).negate(), x.Get(0) }, //
        { Sin.of(angle), Cos.of(angle), x.Get(1) }, //
        { ZeroScalar.get(), ZeroScalar.get(), RealScalar.ONE }, //
    });
  }

  public static Tensor mat2vec(Tensor mat) {
    Scalar arg = ((ArgInterface) ComplexScalar.of(mat.Get(0, 0), mat.Get(1, 0))).arg();
    return Tensors.of(mat.get(0, 2), mat.get(1, 2), arg);
  }

  public static void main(String[] args) {
    Controls c = new Se2Controls(2);
    Flow um1 = c.get(0);
    System.out.println(um1.getU());
    Tensor x = Tensors.vector(0, 0, 0.0);
    Tensor x1 = um1.at(um1.at(x));
    System.out.println(x1);
  }
}
