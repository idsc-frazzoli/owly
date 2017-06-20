// code by marcello
// code adapted by jph
package ch.ethz.idsc.owly.demo.kart;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Hypot;
import ch.ethz.idsc.tensor.sca.ArcTan;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

class MuScriptVbp {
  final Scalar B = RealScalar.of(10);
  final Scalar C = RealScalar.of(1.9);
  final Scalar D = RealScalar.of(1);
  final Scalar eps = RealScalar.of(1e-3);

  MuStruct friction(KartState status, Scalar delta) {
    final Tensor v1 = Tensors.of(status.V, status.dphi.multiply(RimoKart.l_F));
    Scalar V_Fx = v1.dot(status.getCosBsD_SD(delta)).Get();
    Scalar V_Fy = v1.dot(status.getSinBsD_CD(delta)).Get();
    // TODO check why this is not symmetric
    Scalar V_Rx = status.V.multiply(Cos.of(status.beta1));
    Scalar V_Ry = status.V.multiply(Sin.of(status.beta1)) //
        .subtract(status.dphi.multiply(RimoKart.l_R));
    // ---
    Scalar k_FL = slipRatio(status.w_FL, RimoKart.r_FL, V_Fx);
    Scalar k_FR = slipRatio(status.w_FR, RimoKart.r_FR, V_Fx);
    Scalar k_RL = slipRatio(status.w_RL, RimoKart.r_RL, V_Rx);
    Scalar k_RR = slipRatio(status.w_RR, RimoKart.r_RR, V_Rx);
    // ---
    Scalar s_FLX = k_FL.negate().divide(RealScalar.ONE.add(k_FL));
    Scalar s_FLY = V_Fy.divide(status.w_FL.multiply(RimoKart.r_FL));
    //
    Scalar s_FRX = k_FR.negate().divide(RealScalar.ONE.add(k_FR));
    Scalar s_FRY = V_Fy.divide(status.w_FR.multiply(RimoKart.r_FR));
    //
    Scalar s_RLX = k_RL.negate().divide(RealScalar.ONE.add(k_RL));
    Scalar s_RLY = V_Ry.divide(status.w_RL.multiply(RimoKart.r_RL));
    //
    Scalar s_RRX = k_RR.negate().divide(RealScalar.ONE.add(k_RR));
    Scalar s_RRY = V_Ry.divide(status.w_RR.multiply(RimoKart.r_RR));
    // ---
    Scalar s_FL = Hypot.bifunction.apply(s_FLX, s_FLY);
    Scalar s_FR = Hypot.bifunction.apply(s_FRX, s_FRY);
    Scalar s_RL = Hypot.bifunction.apply(s_RLX, s_RLY);
    Scalar s_RR = Hypot.bifunction.apply(s_RRX, s_RRY);
    // ---
    Scalar mu_FL = slipTrig(s_FL);
    Scalar mu_FR = slipTrig(s_FR);
    Scalar mu_RL = slipTrig(s_RL);
    Scalar mu_RR = slipTrig(s_RR);
    // ---
    MuStruct muStruct = new MuStruct();
    //
    muStruct.mu_FLX = process(s_FLX, s_FL, mu_FL);
    muStruct.mu_FLY = process(s_FLY, s_FL, mu_FL);
    //
    muStruct.mu_FRX = process(s_FRX, s_FR, mu_FR);
    muStruct.mu_FRY = process(s_FRY, s_FR, mu_FR);
    //
    muStruct.mu_RLX = process(s_RLX, s_RL, mu_RL);
    muStruct.mu_RLY = process(s_RLY, s_RL, mu_RL);
    //
    muStruct.mu_RRX = process(s_RRX, s_FR, mu_RR);
    muStruct.mu_RRY = process(s_RRY, s_FR, mu_RR);
    //
    return muStruct;
  }

  private Scalar slipTrig(Scalar slip) {
    return D.multiply(Sin.of(C.multiply(ArcTan.of(B.multiply(slip)))));
  }

  private Scalar process(Scalar tslip, Scalar aslip, Scalar mu) {
    return tslip.negate().divide(aslip.add(eps)).multiply(mu);
  }

  private static Scalar slipRatio(Scalar w, Scalar r, Scalar v) {
    return w.multiply(r).subtract(v).divide(v);
  }
}
