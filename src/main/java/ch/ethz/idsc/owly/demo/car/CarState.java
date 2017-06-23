// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public class CarState {
  // TODO not final code design
  static CHatchbackModel params = new CHatchbackModel();
  // ---
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

  // /** @param delta control parameter
  // * @return */
  // Tensor getCosBsD_SD(Scalar delta) {
  // return Tensors.of( //
  // Cos.of(beta1.subtract(delta)), //
  // Sin.of(delta));
  // }
  //
  // /** @param delta control parameter
  // * @return */
  // Tensor getSinBsD_CD(Scalar delta) {
  // return Tensors.of( //
  // Sin.of(beta1.subtract(delta)), //
  // Cos.of(delta));
  // }
  public Tensor asVector() {
    return Tensors.of( //
        Ux, Uy, //
        r, Ksi, px, py, //
        w1L, w1R, w2L, w2R);
  }

  // TODO establish both via matrix mult as vector
  public Scalar getUx1L(Scalar delta) {
    // (Ux - r*params.lw)*cos(delta) + (Uy + r*params.lF)*sin(delta)
    Tensor tang = Tensors.of( //
        Ux.subtract(r.multiply(params.lw())), //
        Uy.add(r.multiply(params.lF())));
    Tensor trig = Tensors.of(Cos.of(delta), Sin.of(delta));
    return tang.dot(trig).Get();
  }

  public Scalar getUy1L(Scalar delta) {
    // -(Ux - r*params.lw)*sin(delta) + (Uy + r*params.lF)*cos(delta);
    Tensor tang = Tensors.of( //
        Ux.subtract(r.multiply(params.lw())).negate(), //
        Uy.add(r.multiply(params.lF())));
    Tensor trig = Tensors.of(Sin.of(delta), Cos.of(delta));
    return tang.dot(trig).Get();
  }

  public Scalar getUx1R(Scalar delta) {
    // (Ux + r*params.lw)*cos(delta) + (Uy + r*params.lF)*sin(delta)
    Tensor tang = Tensors.of( //
        Ux.add(r.multiply(params.lw())), //
        Uy.add(r.multiply(params.lF())));
    Tensor trig = Tensors.of(Cos.of(delta), Sin.of(delta));
    return tang.dot(trig).Get();
  }

  public Scalar getUy1R(Scalar delta) {
    // -(Ux + r*params.lw)*sin(delta) + (Uy + r*params.lF)*cos(delta);
    Tensor tang = Tensors.of( //
        Ux.add(r.multiply(params.lw())).negate(), //
        Uy.add(r.multiply(params.lF())));
    Tensor trig = Tensors.of(Sin.of(delta), Cos.of(delta));
    return tang.dot(trig).Get();
  }

  public Scalar getUx2L() {
    return Ux.subtract(r.multiply(params.lw()));
  }

  public Scalar getUy2L() {
    return Uy.subtract(r.multiply(params.lR()));
  }

  public Scalar getUx2R() {
    return Ux.add(r.multiply(params.lw()));
  }

  public Scalar getUy2R() {
    return Uy.subtract(r.multiply(params.lR()));
  }
}
