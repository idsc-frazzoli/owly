// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.lie.RotationMatrix;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

/** exact integration of exponential function
 * computation in coordinates of lie-algebra */
public enum Se2Integrator implements Integrator {
  INSTANCE;
  @Override
  public Tensor step(Flow flow, Tensor x, Scalar h) {
    Tensor u = flow.getU().multiply(h);
    Tensor ux = Tensors.of(u.Get(1), RealScalar.ZERO, u.Get(0));
    return combine(x, ux);
  }

  /** @param X1 == {px, py, alpha}
   * @param X2 == {vx, vy, beta}
   * @return log [ exp X1 * exp X2 ] */
  public static Tensor combine(Tensor X1, Tensor X2) {
    Scalar al = X1.Get(2);
    Scalar be = X2.Get(2);
    if (Scalars.isZero(be))
      return X1.extract(0, 2).add(RotationMatrix.of(al).dot(X2.extract(0, 2))).append(al);
    Scalar px = X1.Get(0);
    Scalar py = X1.Get(1);
    Scalar vx = X2.Get(0);
    Scalar vy = X2.Get(1);
    Scalar ra = al.add(be);
    Scalar cd = Cos.FUNCTION.apply(ra).subtract(Cos.FUNCTION.apply(al));
    Scalar sd = Sin.FUNCTION.apply(ra).subtract(Sin.FUNCTION.apply(al));
    Scalar rx = px.add(sd.multiply(vx).add(cd.multiply(vy)).divide(be));
    Scalar ry = py.add(sd.multiply(vy).subtract(cd.multiply(vx)).divide(be));
    return Tensors.of(rx, ry, ra);
  }
}
