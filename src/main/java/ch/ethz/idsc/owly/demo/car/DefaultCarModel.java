// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.car.SteeringWheelAngle;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.opt.ConvexHull;
import ch.ethz.idsc.tensor.red.Mean;
import ch.ethz.idsc.tensor.sca.Sign;

public abstract class DefaultCarModel implements VehicleModel {
  // @Override
  // public final Scalar gForce() {
  // return mass().multiply(RealScalar.of(9.81));
  // }
  @Override
  public final int tires() {
    return 4;
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

  public TireInterface tire(int index) {
    throw new RuntimeException();
  }

  @Override
  public Tensor footprint() {
    Tensor hull = Tensors.empty();
    for (int index = 0; index < tires(); ++index)
      hull.append(tire(index).lever().extract(0, 2));
    return ConvexHull.of(hull);
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
    Tensor rear_center = Mean.of(Tensors.vector(i -> tire(2 + i).lever(), 2));
    Tensor p1L = tire(0).lever().subtract(rear_center);
    Tensor p1R = tire(1).lever().subtract(rear_center);
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
    Tensor front_center = Mean.of(Tensors.vector(i -> tire(0 + i).lever(), 2));
    Tensor p2L = tire(2).lever().subtract(front_center);
    Tensor p2R = tire(3).lever().subtract(front_center);
    return Tensors.of( //
        RealScalar.ZERO, // 1L
        RealScalar.ZERO, // 1R
        SteeringWheelAngle.of(p2L.Get(1).divide(p2L.Get(0)), delta.negate()), // 2L
        SteeringWheelAngle.of(p2R.Get(1).divide(p2R.Get(0)), delta.negate()) // 2R
    );
  }

  private Tensor _angles_both(Scalar delta) {
    Tensor p1L = tire(0).lever();
    Tensor p1R = tire(1).lever();
    Tensor p2L = tire(2).lever();
    Tensor p2R = tire(3).lever();
    return Tensors.of( //
        SteeringWheelAngle.of(p1L.Get(1).divide(p1L.Get(0)), delta), //
        SteeringWheelAngle.of(p1R.Get(1).divide(p1R.Get(0)), delta), //
        SteeringWheelAngle.of(p2L.Get(1).divide(p2L.Get(0)), delta.negate()), //
        SteeringWheelAngle.of(p2R.Get(1).divide(p2R.Get(0)), delta.negate()) //
    );
  }
}
