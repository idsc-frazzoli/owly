// code by edo
package ch.ethz.idsc.owly.demo.drift;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.ArcTan;
import ch.ethz.idsc.tensor.sca.Sin;
import ch.ethz.idsc.tensor.sca.Sqrt;

public class DriftStateSpaceModel implements StateSpaceModel {
  private final DriftParameters driftParameters;

  public DriftStateSpaceModel(DriftParameters driftParameters) {
    this.driftParameters = driftParameters;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    Scalar beta = x.Get(0);
    Scalar r = x.Get(1);
    Scalar Ux = x.Get(2);
    Scalar delta = u.Get(0);
    Scalar FxR = u.Get(1);
    Scalar FyF = Fy_F(beta, r, Ux, delta);
    Scalar FyR = Fy_R(beta, r, Ux, FxR);
    Scalar dbeta = FyF.add(FyR).divide(driftParameters.m.multiply(Ux)).subtract(r);
    Scalar dr = driftParameters.a.multiply(FyF).subtract(driftParameters.b.multiply(FyR)).divide(driftParameters.Iz);
    // Scalar dUx = (FxR - FyF * sin(delta)) / params.m + r * Ux * beta;
    Scalar dUx = FxR.subtract(FyF.multiply(Sin.of(delta))).divide(driftParameters.m).add(r.multiply(Ux.multiply(beta)));
    return Tensors.of(dbeta, dr, dUx);
  }

  public Scalar Fy_F(Scalar beta, Scalar r, Scalar Ux, Scalar delta) {
    Scalar aF = a_F(beta, r, Ux, delta);
    Scalar FzF = driftParameters.Fz_F();
    return pacejka(aF, RealScalar.ZERO, FzF, driftParameters.muF);
  }

  public Scalar Fy_R(Scalar beta, Scalar r, Scalar Ux, Scalar FxR) {
    Scalar aR = a_R(beta, r, Ux);
    Scalar FzR = driftParameters.Fz_R();
    return pacejka(aR, FxR, FzR, driftParameters.muR);
  }

  public Scalar a_F(Scalar beta, Scalar r, Scalar Ux, Scalar delta) {
    // TODO put here atan2
    return ArcTan.of(beta.add(driftParameters.a.multiply(r).divide(Ux))).subtract(delta);
  }

  public Scalar a_R(Scalar beta, Scalar r, Scalar Ux) {
    // TODO put here atan2
    return ArcTan.of(beta.subtract(driftParameters.b.multiply(r).divide(Ux)));
  }

  public Scalar pacejka(Scalar slip, Scalar Fx, Scalar Fz, Scalar mu) {
    Scalar eps;
    final Scalar muFz = mu.multiply(Fz);
    if (Scalars.lessThan(muFz, Fx))
      eps = RealScalar.ZERO;
    else
      eps = Sqrt.of(muFz.multiply(muFz).subtract(Fx.multiply(Fx))).divide(muFz);
    Scalar Fpay = eps.multiply(muFz).multiply(driftParameters.pacejka3.apply(slip)).negate();
    return Fpay;
  }

  @Override
  public Scalar getLipschitz() {
    // TODO Auto-generated method stub
    return null;
  }
}
