// code by jph
package ch.ethz.idsc.owly.math.car;

import ch.ethz.idsc.tensor.NumberQ;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.sca.ArcTan;
import ch.ethz.idsc.tensor.sca.ScalarUnaryOperator;
import ch.ethz.idsc.tensor.sca.Sin;

/** The Pacejka "Magic Formula" tire models
 * 
 * Pacejka3 depends on 3 parameters: B, C, D
 * 
 * https://en.wikipedia.org/wiki/Hans_B._Pacejka */
public class Pacejka3 implements ScalarUnaryOperator {
  public final Scalar B;
  public final Scalar C;
  public final Scalar D;

  public Pacejka3(double B, double C, double D) {
    this.B = RealScalar.of(B);
    this.C = RealScalar.of(C);
    this.D = RealScalar.of(D);
  }

  @Override
  public Scalar apply(Scalar slip) {
    if (!NumberQ.of(slip))
      throw TensorRuntimeException.of(slip);
    return D.multiply(Sin.of(C.multiply(ArcTan.of(B.multiply(slip)))));
  }
}
