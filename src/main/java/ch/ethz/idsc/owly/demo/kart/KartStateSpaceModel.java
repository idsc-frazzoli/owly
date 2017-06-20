// code by marcello
// code adapted by jph
package ch.ethz.idsc.owly.demo.kart;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** the corresponding Matlab function is
 * kart_vbp(t, u, status, k)
 * 
 * u = {accel, delta_instantaneous} */
public class KartStateSpaceModel implements StateSpaceModel {
  @Override
  public Tensor f(Tensor x, Tensor u) {
    // Scalar accel = u.Get(0);
    Scalar T_FL = u.Get(0);
    Scalar T_FR = u.Get(0);
    Scalar T_RL = u.Get(0);
    Scalar T_RR = u.Get(0);
    Scalar delta = u.Get(1);
    Scalar h = RealScalar.of(.5);
    return null;
  }

  @Override
  public Scalar getLipschitz() {
    // TODO Auto-generated method stub
    return null;
  }
}
