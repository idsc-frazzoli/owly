// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.Cross2D;
import ch.ethz.idsc.owly.math.FrictionCoefficients;
import ch.ethz.idsc.owly.math.PhysicalConstants;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Join;
import ch.ethz.idsc.tensor.lie.Cross;
import ch.ethz.idsc.tensor.lie.Rodriguez;
import ch.ethz.idsc.tensor.lie.RotationMatrix;
import ch.ethz.idsc.tensor.mat.LinearSolve;
import ch.ethz.idsc.tensor.red.Total;
import ch.ethz.idsc.tensor.sca.Chop;

/** implementation has been verified through several tests */
public class TireForces {
  private static final Tensor AFFINE_ONE = Tensors.vector(1);
  private static final Tensor SUM_ALL = Tensors.vector(1, 1, 1, 1).unmodifiable();
  private static final Tensor WEIGHT_STD = Tensors.vector(+1, -1, -1, +1).unmodifiable();
  // ---
  public final VehicleModel params;
  public final CarState cs;
  public final Tensor Forces; // forces in car/body frame
  public final Tensor fwheel; // forces in wheel frame

  /** @param params
   * @param cs
   * @param cc
   * @param mu friction coefficient of tire on road/ground, see {@link FrictionCoefficients} */
  public TireForces(VehicleModel params, CarState cs, CarControl cc, Scalar mu) {
    this.params = params;
    this.cs = cs;
    final Tensor angles = params.angles(cc.delta).unmodifiable();
    // ---
    Tensor mus = Tensors.vector(index -> //
    new RobustSlip(params.tire(index).pacejka(), get_ui_2d(angles.Get(index), index), //
        params.tire(index).radius().multiply(cs.omega.Get(index))).slip(), 4).multiply(mu);
    final Tensor dir = Tensors.vector(index -> //
    Join.of(RotationMatrix.of(angles.Get(index)).dot(mus.get(index)), AFFINE_ONE), 4);
    // ---
    final Tensor fbodyZ;
    {
      Tensor levers = Tensors.vector(i -> params.tire(i).lever(), 4);
      Tensor rotX_z = levers.get(Tensor.ALL, 1);
      Tensor rotX_y = levers.get(Tensor.ALL, 2).pmul(dir.get(Tensor.ALL, 1)); // z coordinate of tire contact * dir_y
      Tensor rotY_z = levers.get(Tensor.ALL, 0);
      Tensor rotY_x = levers.get(Tensor.ALL, 2).pmul(dir.get(Tensor.ALL, 0)); // z coordinate of tire contact * dir_x
      Tensor Lhs = Tensors.of( //
          rotX_z.subtract(rotX_y), // no rotation around X
          rotY_z.subtract(rotY_x), // no rotation around Y
          SUM_ALL, // compensate g-force
          WEIGHT_STD // weight transfer TODO geometry of COG?
      );
      // System.out.println("det=" + Det.of(Lhs));
      Tensor rhs = Array.zeros(4);
      Scalar gForce = params.mass().multiply(PhysicalConstants.G_EARTH);
      rhs.set(gForce, 2);
      fbodyZ = LinearSolve.of(Lhs, rhs);
    }
    // ---
    // if (false) {
    // Scalar lF = params.levers().Get(0, 0);
    // Scalar lR = params.levers().Get(2, 0).negate();
    // Scalar lR_lF = lF.add(lR).multiply(RealScalar.of(2));
    // Fz1L = lR.divide(lR_lF).multiply(params.gForce());
    // Fz1R = lR.divide(lR_lF).multiply(params.gForce());
    // Fz2L = lF.divide(lR_lF).multiply(params.gForce());
    // Fz2R = lF.divide(lR_lF).multiply(params.gForce());
    // }
    fwheel = fbodyZ.pmul(mus).unmodifiable();
    Forces = fbodyZ.pmul(dir).unmodifiable();
  }

  /** @return torque on vehicle at center of mass */
  public Tensor torque() {
    Tensor tensor = Array.zeros(3);
    for (int index = 0; index < params.tires(); ++index)
      tensor = tensor.add(Cross.of(params.tire(index).lever(), Forces.get(index)));
    return tensor;
  }

  public boolean isTorqueConsistent() {
    return Chop._07.allZero(torque().extract(0, 2));
  }

  public boolean isFzConsistent() {
    Scalar f03 = Forces.Get(0, 2).add(Forces.Get(3, 2));
    Scalar f12 = Forces.Get(1, 2).add(Forces.Get(2, 2));
    return Chop._07.close(f03, f12);
  }

  public boolean isGForceConsistent() {
    Scalar gForce = params.mass().multiply(PhysicalConstants.G_EARTH);
    return Chop._07.close(Total.of(Forces).Get(2), gForce);
  }

  /** @param delta angle of wheel
   * @param index of wheel
   * @return */
  private Tensor get_ui_2d(Scalar delta, int index) { // as in doc
    Tensor tangent_2 = cs.u_2d().add(Cross2D.of(params.tire(index).lever().extract(0, 2).multiply(cs.r)));
    return RotationMatrix.of(delta.negate()).dot(tangent_2);
  }

  /** implementation below is for full 3d rotations, but not used since
   * at the moment our car rotates in plane (only around z-axis)
   * 
   * @param delta
   * @param index
   * @return */
  /* package */ Tensor get_ui_3(Scalar delta, int index) { // as in doc
    Tensor rotation_3 = Rodriguez.of(Tensors.of(RealScalar.ZERO, RealScalar.ZERO, delta.negate()));
    Tensor tangent_3 = cs.u_3d().add(Cross.of(cs.rate_3d(), params.tire(index).lever()));
    return rotation_3.dot(tangent_3).extract(0, 2);
  }

  /***************************************************/
  // FOR TESTS ONLY
  Tensor asVectorFX() { // Tensors.of(Fx1L, Fx1R, Fx2L, Fx2R);
    return Forces.get(Tensor.ALL, 0);
  }

  Tensor asVectorFY() { // Tensors.of(Fy1L, Fy1R, Fy2L, Fy2R);
    return Forces.get(Tensor.ALL, 1);
  }

  Tensor asVectorFZ() { // Tensors.of(Fz1L, Fz1R, Fz2L, Fz2R);
    return Forces.get(Tensor.ALL, 2);
  }

  Tensor asVector_fX() { // Tensors.of(fx1L, fx1R, fx2L, fx2R);
    return fwheel.get(Tensor.ALL, 0);
  }

  Tensor asVector_fY() { // Tensors.of(fy1L, fy1R, fy2L, fy2R);
    return fwheel.get(Tensor.ALL, 1);
  }
}
