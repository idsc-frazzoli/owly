// code by jph
package ch.ethz.idsc.owly.demo.kart;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public class KartState {
  public final Scalar V;
  public final Scalar beta1;
  public final Scalar phi;
  public final Scalar dphi;
  public final Scalar w_FL;
  public final Scalar w_FR;
  public final Scalar w_RL;
  public final Scalar w_RR;
  public final Scalar x_BF;
  public final Scalar y_BF;

  public KartState(Tensor vector) {
    V = vector.Get(0); // V
    beta1 = vector.Get(1); // beta1
    phi = vector.Get(2); // phi
    dphi = vector.Get(3);
    w_FL = vector.Get(4); // w_FL
    w_FR = vector.Get(5); // w_FR
    w_RL = vector.Get(6);
    w_RR = vector.Get(7);
    x_BF = vector.Get(8); // body frame position
    y_BF = vector.Get(9); // body frame position
  }

  /** @param delta control parameter
   * @return */
  Tensor getCosBsD_SD(Scalar delta) {
    return Tensors.of( //
        Cos.of(beta1.subtract(delta)), //
        Sin.of(delta));
  }

  /** @param delta control parameter
   * @return */
  Tensor getSinBsD_CD(Scalar delta) {
    return Tensors.of( //
        Sin.of(beta1.subtract(delta)), //
        Cos.of(delta));
  }
}
