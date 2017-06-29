// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.car.SteeringWheelAngle;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.red.Mean;
import ch.ethz.idsc.tensor.sca.Sign;

public abstract class DefaultCarModel implements CarModel {
  @Override
  public final Scalar gForce() {
    return mass().multiply(RealScalar.of(9.81));
  }

  @Override
  public final Scalar radiusTimes(Scalar omega) {
    return radius().multiply(omega);
  }

  @Override
  public final Scalar noSlipRate(Scalar speed) {
    return speed.divide(radius());
  }

  @Override
  public final Scalar coulombFriction(Scalar speed) {
    return Sign.of(speed).multiply(b().multiply(speed.abs()).add(fric()));
  }

  @Override
  public final Tensor angles(Scalar delta) {
    switch (steering()) {
    case FRONT:
      return _angles_front(delta);
    case FRONT_PARALLEL:
      return _angles_frontParallel(delta);
    case REAR:
      return _angles_rear(delta);
    case BOTH:
      return _angles_both(delta);
    default:
      break;
    }
    return null;
  }

  /***************************************************/
  /** @return dynamic friction coefficient N/(m/s) */
  public abstract Scalar b();

  /** @return coulomb friction */
  public abstract Scalar fric();

  /** @return */
  public abstract CarSteering steering();

  /***************************************************/
  // helper functions
  private Tensor _angles_front(Scalar delta) {
    Tensor rear_center = Mean.of(levers().extract(2, 4));
    Tensor p1L = levers().get(0).subtract(rear_center);
    Tensor p1R = levers().get(1).subtract(rear_center);
    return Tensors.of( //
        SteeringWheelAngle.of(p1L.Get(1).divide(p1L.Get(0)), delta), // 1L
        SteeringWheelAngle.of(p1R.Get(1).divide(p1R.Get(0)), delta), // 1R
        RealScalar.ZERO, // 2L
        RealScalar.ZERO // 2R
    );
  }

  private Tensor _angles_frontParallel(Scalar delta) {
    return Tensors.of( //
        delta, // 1L
        delta, // 1R
        RealScalar.ZERO, // 2L
        RealScalar.ZERO // 2R
    );
  }

  private Tensor _angles_rear(Scalar delta) {
    Tensor front_center = Mean.of(levers().extract(0, 1));
    Tensor p2L = levers().get(2).subtract(front_center);
    Tensor p2R = levers().get(3).subtract(front_center);
    return Tensors.of( //
        RealScalar.ZERO, // 1L
        RealScalar.ZERO, // 1R
        SteeringWheelAngle.of(p2L.Get(1).divide(p2L.Get(0)), delta.negate()), // 2L
        SteeringWheelAngle.of(p2R.Get(1).divide(p2R.Get(0)), delta.negate()) // 2R
    );
  }

  private Tensor _angles_both(Scalar delta) {
    Tensor center = Array.zeros(3);
    Tensor p1L = levers().get(0).subtract(center); // TODO simplify
    Tensor p1R = levers().get(1).subtract(center);
    Tensor p2L = levers().get(2).subtract(center);
    Tensor p2R = levers().get(3).subtract(center);
    return Tensors.of( //
        SteeringWheelAngle.of(p1L.Get(1).divide(p1L.Get(0)), delta), //
        SteeringWheelAngle.of(p1R.Get(1).divide(p1R.Get(0)), delta), //
        SteeringWheelAngle.of(p2L.Get(1).divide(p2L.Get(0)), delta.negate()), //
        SteeringWheelAngle.of(p2R.Get(1).divide(p2R.Get(0)), delta.negate()) //
    );
  }
}
