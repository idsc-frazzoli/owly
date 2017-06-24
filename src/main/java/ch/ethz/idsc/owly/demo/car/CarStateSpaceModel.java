// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public class CarStateSpaceModel implements StateSpaceModel {
  private final CHatchbackModel params; // TODO not final design

  public CarStateSpaceModel(CarModel params) {
    this.params = (CHatchbackModel) params; // TODO
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    if (x.length() != 10)
      throw TensorRuntimeException.of(x);
    if (u.length() != 4)
      throw TensorRuntimeException.of(u);
    // ---
    // TODO apply u rate limiter
    // ---
    CarState cs = new CarState(x);
    CarControl cc = new CarControl(u);
    // [ FORCES, forces] = tires(x,u);
    TireForces tire = new TireForces(params, cs, cc);
    BrakeTorques brakeTorques = new BrakeTorques(cs, cc, tire);
    MotorTorques torques = new MotorTorques(params, cc.throttle);
    // ---
    //
    Scalar du; // TODO rename du, dv
    {
      Scalar prel = tire.total1234().add(params.mass().multiply(cs.Uy).multiply(cs.r));
      // TODO rollFric
      du = prel.subtract(params.coulombFriction(cs.Ux)).divide(params.mass());
    }
    // ---
    Scalar dv;
    {
      Scalar prel = tire.total5678().subtract(params.mass().multiply(cs.Ux).multiply(cs.r));
      // TODO rollFric
      dv = prel.subtract(RealScalar.ZERO.multiply(params.coulombFriction(cs.Uy))).divide(params.mass());
    }
    // ---
    //
    Scalar dr;
    {
      Tensor vec1 = Tensors.of(params.lF(), params.lR().negate(), params.lw());
      Tensor vec2 = Tensors.of(tire.total56(), tire.total78(), tire.total24_13());
      dr = vec1.dot(vec2).Get().multiply(params.Iz_invert());
    }
    Scalar dKsi = cs.r;
    Scalar dx = cs.Ux.multiply(Cos.of(cs.Ksi)).subtract(cs.Uy.multiply(Sin.of(cs.Ksi)));
    Scalar dy = cs.Ux.multiply(Sin.of(cs.Ksi)).add(cs.Uy.multiply(Cos.of(cs.Ksi)));
    Scalar dw1L = torques.Tm1L.add(brakeTorques.Tb1L).subtract(params.radiusTimes(tire.fx1L)).multiply(params.Iw_invert());
    Scalar dw1R = torques.Tm1R.add(brakeTorques.Tb1R).subtract(params.radiusTimes(tire.fx1R)).multiply(params.Iw_invert());
    Scalar dw2L = torques.Tm2L.add(brakeTorques.Tb2L).subtract(params.radiusTimes(tire.fx2L)).multiply(params.Iw_invert());
    Scalar dw2R = torques.Tm2R.add(brakeTorques.Tb2R).subtract(params.radiusTimes(tire.fx2R)).multiply(params.Iw_invert());
    //
    return Tensors.of(du, dv, dr, dKsi, dx, dy, dw1L, dw1R, dw2L, dw2R);
  }

  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE; // null
  }
}
