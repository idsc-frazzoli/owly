// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;

public class CarState {
  public final Scalar Ux; // 1 long speed body frame [m/s]
  public final Scalar Uy; // 2 lateral speed body frame [m/s]
  public final Scalar r; // 3 yawing rate [rad/s]
  public final Scalar Ksi; // 4 heading of the car [rad]
  public final Scalar px; // 5 pos [m]
  public final Scalar py; // 6 pos [m]
  public final Scalar w1L; // [rad/s]
  public final Scalar w1R; // [rad/s]
  public final Scalar w2L; // [rad/s]
  public final Scalar w2R; // [rad/s]

  public CarState(Tensor x) {
    if (x.length() != 10)
      throw TensorRuntimeException.of(x);
    // ---
    Ux = x.Get(0);
    Uy = x.Get(1);
    r = x.Get(2);
    Ksi = x.Get(3);
    px = x.Get(4);
    py = x.Get(5);
    w1L = x.Get(6);
    w1R = x.Get(7);
    w2L = x.Get(8);
    w2R = x.Get(9);
  }

  /** @return state encoded as vector for input to {@link StateSpaceModel} */
  public Tensor asVector() {
    return Tensors.of( //
        Ux, Uy, //
        r, Ksi, px, py, //
        w1L, w1R, w2L, w2R);
  }

  public Tensor u_2d() {
    return Tensors.of(Ux, Uy);
  }

  public Tensor u_3d() {
    return Tensors.of(Ux, Uy, RealScalar.ZERO);
  }

  public Tensor rate_3d() {
    return Tensors.of(RealScalar.ZERO, RealScalar.ZERO, r);
  }
}
