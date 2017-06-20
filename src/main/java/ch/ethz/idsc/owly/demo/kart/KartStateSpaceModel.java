// code by marcello
// code adapted by jph
package ch.ethz.idsc.owly.demo.kart;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

/** the corresponding Matlab function is
 * kart_vbp(t, u, status, k)
 * 
 * u = {accel, delta_instantaneous} */
public class KartStateSpaceModel implements StateSpaceModel {
  private final Scalar timeStep; // TODO

  public KartStateSpaceModel(Scalar timeStep) {
    this.timeStep = timeStep;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    // Scalar accel = u.Get(0);
    Scalar T_FL = u.Get(0);
    Scalar T_FR = u.Get(0);
    Scalar T_RL = u.Get(0);
    Scalar T_RR = u.Get(0);
    Scalar delta = u.Get(1);
    Scalar h = RealScalar.of(.5);
    KartState status = new KartState(x);
    ForcesVbp forcesVbp = ForcesVbp.compute(h, status, delta, timeStep);
    Scalar f_FX = forcesVbp.f_FX();
    Scalar f_RX = forcesVbp.f_RX();
    Scalar f_FY = forcesVbp.f_FY();
    Scalar f_RY = forcesVbp.f_RY();
    // ---
    Scalar dV = RealScalar.ZERO;
    dV = dV.add(f_FX.multiply(Cos.of(delta.subtract(status.beta1))));
    dV = dV.subtract(f_FY.multiply(Sin.of(delta.subtract(status.beta1))));
    dV = dV.add(f_RX.multiply(Cos.of(status.beta1)));
    dV = dV.add(f_RY.multiply(Sin.of(status.beta1)));
    dV = dV.multiply(RimoKart.m);
    // ---
    Scalar dbeta = RealScalar.ZERO;
    dbeta = dbeta.add(f_FX.multiply(Sin.of(delta.subtract(status.beta1))));
    dbeta = dbeta.add(f_FY.multiply(Cos.of(delta.subtract(status.beta1))));
    dbeta = dbeta.subtract(f_RX.multiply(Sin.of(status.beta1)));
    dbeta = dbeta.add(f_RY.multiply(Cos.of(status.beta1)));
    dbeta = dbeta.divide(RimoKart.m.multiply(status.V));
    dbeta = dbeta.subtract(status.dphi);
    // ---
    Scalar ddphi = RealScalar.ZERO;
    ddphi = ddphi.add(RimoKart.l_F.multiply( //
        f_FY.multiply(Cos.of(delta)).add(f_FX.multiply(Sin.of(delta)))));
    ddphi = ddphi.subtract(RimoKart.l_R.multiply( //
        f_RY)); // check again
    ddphi = ddphi.add(RimoKart.w_L.multiply( //
        forcesVbp.f_FLY.multiply(Sin.of(delta)) //
            .subtract(forcesVbp.f_FLX.multiply(Cos.of(delta))) //
            .subtract(forcesVbp.f_RLX)));
    ddphi = ddphi.add(RimoKart.w_R.multiply( //
        forcesVbp.f_FRX.multiply(Cos.of(delta)) //
            .subtract(forcesVbp.f_FRY.multiply(Sin.of(delta))) //
            .add(forcesVbp.f_RRX)));
    ddphi = ddphi.divide(RimoKart.I_z);
    // ---
    Scalar dw_FL = T_FL.subtract(forcesVbp.f_FLX.multiply(RimoKart.r_FL)).divide(RimoKart.I_w);
    Scalar dw_FR = T_FR.subtract(forcesVbp.f_FRX.multiply(RimoKart.r_FR)).divide(RimoKart.I_w);
    Scalar dw_RL = T_RL.subtract(forcesVbp.f_RLX.multiply(RimoKart.r_RL)).divide(RimoKart.I_w);
    Scalar dw_RR = T_RR.subtract(forcesVbp.f_RRX.multiply(RimoKart.r_RR)).divide(RimoKart.I_w);
    // ---
    Scalar Vx_bf = status.V.multiply(Cos.of(status.beta1));
    Scalar Vy_bf = status.V.multiply(Sin.of(status.beta1));
    Tensor V_VF = toWorldF(Vx_bf, Vy_bf, status.phi);
    Scalar x_WF = V_VF.Get(0);
    Scalar y_WF = V_VF.Get(1);
    return Tensors.of( //
        dV, dbeta, status.dphi, ddphi, dw_FL, dw_FR, dw_RL, dw_RR, x_WF, y_WF);
  }

  private static Tensor toWorldF(Scalar vx_bf, Scalar vy_bf, Scalar phi) {
    Tensor R = Tensors.of( //
        Tensors.of(Cos.of(phi), Sin.of(phi).negate()), //
        Tensors.of(Sin.of(phi), Cos.of(phi)));
    return R.dot(Tensors.of(vx_bf, vy_bf));
  }

  @Override
  public Scalar getLipschitz() {
    // TODO Auto-generated method stub
    return null;
  }
}
