// code by marcello
// code adapted by jph
package ch.ethz.idsc.owly.demo.kart;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

class WeightTransfer {
  Scalar Vx_prev = null; // FIXME
  Scalar Vy_prev = null;

  public ForcesZ compute(Scalar h, KartState status, Scalar delta, Scalar timeStep) {
    if (Vx_prev == null) {
      Vx_prev = status.V.multiply(Cos.of(status.beta1));
      Vy_prev = status.V.multiply(Sin.of(status.beta1));
    }
    // numerical gradient to compute acceleration
    Scalar Vx = status.V.multiply(Cos.of(status.beta1));
    Scalar Vy = status.V.multiply(Sin.of(status.beta1));
    Scalar a_x = Vx.subtract(Vx_prev).multiply(timeStep);
    Scalar a_y = Vy.subtract(Vy_prev).multiply(timeStep);
    // for next time step
    Vx_prev = Vx;
    Vy_prev = Vy;
    // ---
    Scalar f0_FLZ = RimoKart.f0_FLZ();
    Scalar f0_FRZ = RimoKart.f0_FRZ();
    Scalar f0_RLZ = RimoKart.f0_RLZ();
    Scalar f0_RRZ = RimoKart.f0_RRZ();
    // ---
    Scalar delta_ffy = RimoKart.d_ffy(h, a_y);
    Scalar delta_fry = RimoKart.d_fry(h, a_y);
    Scalar delta_flx = RimoKart.d_flx(h, a_x);
    Scalar delta_frx = RimoKart.d_frx(h, a_x);
    // ---
    ForcesZ forcesZ = new ForcesZ();
    forcesZ.f_FLZ = f0_FLZ.subtract(delta_flx).subtract(delta_ffy);
    forcesZ.f_FRZ = f0_FRZ.subtract(delta_frx).add(delta_ffy);
    forcesZ.f_RLZ = f0_RLZ.add(delta_flx).subtract(delta_fry);
    forcesZ.f_RRZ = f0_RRZ.add(delta_frx).add(delta_fry);
    return forcesZ;
  }
}
