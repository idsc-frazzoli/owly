// code by jph
package ch.ethz.idsc.owly.demo.kart;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public class MuScriptVbp {
  public MuScriptVbp(KartState status, Scalar delta) {
    final Tensor v1 = Tensors.of(status.V, status.dphi.multiply(RimoKart.l_F));
    Scalar V_Fx = v1.dot(status.getCosBsD_SD(delta)).Get();
    Scalar V_Fy = v1.dot(status.getSinBsD_CD(delta)).Get();
    // TODO check why this is not symmetric
    Scalar V_Rx = status.V.multiply(Cos.of(status.beta1));
    Scalar V_Ry = status.V.multiply(Sin.of(status.beta1)) //
        .subtract(status.dphi.multiply(RimoKart.l_R));
    // ---
    // Scalar k_FL = 12;
    // Scalar V_Fx = status.V.multiply(scalar)
    // RimoKart.l_F.multiply(scalar)
  }

  private void init() {
  }
}
