// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.Deadzone;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.mat.RotationMatrix;

/** the matlab code applies a rate limiter to u
 * if this is beneficial for stability, the limiter should
 * be a layer outside of the state space model */
public class CarStateSpaceModel implements StateSpaceModel {
  private final CarModel params;

  public CarStateSpaceModel(CarModel carModel) {
    this.params = carModel;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    // TODO u may need to satisfy certain conditions with respect to previous u
    CarState cs = new CarState(x);
    CarControl cc = new CarControl(u);
    TireForces tire = new TireForces(params, cs, cc);
    BrakeTorques brakeTorques = new BrakeTorques(params, cs, cc, tire);
    MotorTorques torques = new MotorTorques(params, cc.throttle);
    // ---
    final Scalar dux;
    final Scalar rollFric = params.rollFric(); // TODO at the moment == 0!
    Deadzone deadzone = Deadzone.of(rollFric.negate(), rollFric);
    {
      Scalar prel = tire.total1234().add(params.mass().multiply(cs.Uy).multiply(cs.r));
      dux = deadzone.apply(prel).subtract(params.coulombFriction(cs.Ux)).divide(params.mass());
    }
    // ---
    final Scalar duy;
    {
      Scalar prel = tire.total5678().subtract(params.mass().multiply(cs.Ux).multiply(cs.r));
      duy = deadzone.apply(prel).subtract(RealScalar.ZERO.multiply(params.coulombFriction(cs.Uy))).divide(params.mass());
    }
    // ---
    Scalar dr;
    {
      Tensor vec1 = Tensors.of(params.lF(), params.lR().negate(), params.lw());
      Tensor vec2 = Tensors.of(tire.total56(), tire.total78(), tire.total24_13());
      dr = vec1.dot(vec2).Get().multiply(params.Iz_invert());
    }
    Tensor dp = RotationMatrix.of(cs.Ksi).dot(cs.groundSpeed());
    // ---
    Scalar dw1L = torques.Tm1L.add(brakeTorques.Tb1L).subtract(params.radiusTimes(tire.fx1L)).multiply(params.Iw_invert());
    Scalar dw1R = torques.Tm1R.add(brakeTorques.Tb1R).subtract(params.radiusTimes(tire.fx1R)).multiply(params.Iw_invert());
    Scalar dw2L = torques.Tm2L.add(brakeTorques.Tb2L).subtract(params.radiusTimes(tire.fx2L)).multiply(params.Iw_invert());
    Scalar dw2R = torques.Tm2R.add(brakeTorques.Tb2R).subtract(params.radiusTimes(tire.fx2R)).multiply(params.Iw_invert());
    // ---
    return Tensors.of( //
        dux, duy, //
        dr, //
        cs.r, //
        dp.Get(0), dp.Get(1), //
        dw1L, dw1R, dw2L, dw2R);
  }

  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE; // null
  }
}
