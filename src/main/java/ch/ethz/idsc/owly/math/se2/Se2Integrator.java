// code by jph
package ch.ethz.idsc.owly.math.se2;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.flow.RungeKutta4Integrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.lie.RotationMatrix;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

/** exact integration of flow using matrix exponential and logarithm.
 * states are encoded in the default coordinates of the se2 lie-algebra.
 * 
 * Se2Integrator is approximately
 * 3x faster than {@link RungeKutta4Integrator}
 * 11x faster than {@link RungeKutta45Integrator} */
public enum Se2Integrator implements Integrator {
  INSTANCE;
  // ---
  @Override
  public Tensor step(Flow flow, Tensor x, Scalar h) {
    final Tensor u = flow.getU();
    Scalar speed = u.Get(1).multiply(h);
    Tensor ux = Tensors.of(speed, RealScalar.ZERO, u.Get(0).multiply(speed));
    return combine_vy0(x, ux);
  }

  /** function integrates the special case where the y-component of X2
   * is constrained to equal 0.
   * 
   * @param X1 == {px, py, alpha}
   * @param X2 == {vx, 0, beta}
   * @return log [ exp X1 * exp X2 ] */
  public static Tensor combine_vy0(Tensor X1, Tensor X2) {
    Scalar al = X1.Get(2);
    Scalar be = X2.Get(2);
    if (Scalars.isZero(be))
      return X1.extract(0, 2).add(RotationMatrix.of(al).dot(X2.extract(0, 2))).append(al);
    Scalar px = X1.Get(0);
    Scalar py = X1.Get(1);
    Scalar vx = X2.Get(0);
    Scalar ra = al.add(be);
    Scalar cd = Cos.FUNCTION.apply(ra).subtract(Cos.FUNCTION.apply(al));
    Scalar sd = Sin.FUNCTION.apply(ra).subtract(Sin.FUNCTION.apply(al));
    return Tensors.of( //
        px.add(sd.multiply(vx).divide(be)), //
        py.subtract(cd.multiply(vx).divide(be)), //
        ra);
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
    return Tensors.of( //
        px.add(sd.multiply(vx).add(cd.multiply(vy)).divide(be)), //
        py.add(sd.multiply(vy).subtract(cd.multiply(vx)).divide(be)), //
        ra);
  }

  /** @param X2 == {vx, vy, beta}
   * @return very confused I am */
  public static Tensor combine0(Tensor X2) {
    Scalar be = X2.Get(2);
    if (Scalars.isZero(be))
      return X2.extract(0, 2).append(RealScalar.ZERO);
    Scalar vx = X2.Get(0);
    Scalar vy = X2.Get(1);
    Scalar cd = Cos.FUNCTION.apply(be).subtract(RealScalar.ONE);
    Scalar sd = Sin.FUNCTION.apply(be);
    return Tensors.of( //
        sd.multiply(vx).add(cd.multiply(vy)).divide(be), //
        sd.multiply(vy).subtract(cd.multiply(vx)).divide(be), //
        be);
  }
}
