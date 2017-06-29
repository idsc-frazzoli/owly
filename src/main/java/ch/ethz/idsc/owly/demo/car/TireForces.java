// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.Cross2D;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Transpose;
import ch.ethz.idsc.tensor.lie.Cross;
import ch.ethz.idsc.tensor.lie.Rodriguez;
import ch.ethz.idsc.tensor.mat.RotationMatrix;
import ch.ethz.idsc.tensor.red.Total;

/** implementation has been verified through several tests */
public class TireForces {
  public final CarModel params;
  public final CarState cs;
  public final Tensor Forces; // forces in car/body frame
  public final Tensor fwheel; // forces in wheel frame

  public TireForces(CarModel params, CarState cs, CarControl cc) {
    this.params = params;
    this.cs = cs;
    final Tensor angles = params.angles(cc.delta).unmodifiable();
    //
    final Tensor _u1L = get_ui_2d(angles.Get(0), 0);
    final SlipInterface mu1L = new ReducedSlip(params.pacejka1(), params.mu(), _u1L, params.radiusTimes(cs.w1L));
    final Tensor _u1R = get_ui_2d(angles.Get(1), 1);
    final SlipInterface mu1R = new ReducedSlip(params.pacejka1(), params.mu(), _u1R, params.radiusTimes(cs.w1R));
    final Tensor _u2L = get_ui_2d(angles.Get(2), 2);
    final SlipInterface mu2L = new ReducedSlip(params.pacejka2(), params.mu(), _u2L, params.radiusTimes(cs.w2L));
    final Tensor _u2R = get_ui_2d(angles.Get(3), 3);
    final SlipInterface mu2R = new ReducedSlip(params.pacejka2(), params.mu(), _u2R, params.radiusTimes(cs.w2R));
    // ---
    final Scalar h = params.heightCog();
    final Tensor ck1L = RotationMatrix.of(angles.Get(0)).dot(mu1L.slip()).multiply(h);
    final Tensor ck1R = RotationMatrix.of(angles.Get(1)).dot(mu1R.slip()).multiply(h);
    final Tensor ck2L = RotationMatrix.of(angles.Get(2)).dot(mu2L.slip()).multiply(h);
    final Tensor ck2R = RotationMatrix.of(angles.Get(3)).dot(mu2R.slip()).multiply(h);
    // ---
    Tensor EA = ck1L.add(params.levers().get(0).extract(0, 2)); // changed from "subtract" to "add"
    Tensor FB = ck1R.add(params.levers().get(1).extract(0, 2));
    Tensor GC = ck2L.add(params.levers().get(2).extract(0, 2));
    Tensor HD = ck2R.add(params.levers().get(3).extract(0, 2));
    // ---
    final Scalar den = Total.of(Tensors.of( //
        EA.dot(Cross2D.of(FB)), //
        FB.dot(Cross2D.of(HD)), //
        GC.dot(Cross2D.of(EA)), //
        HD.dot(Cross2D.of(GC)) //
    )).multiply(RealScalar.of(2)).Get();
    //
    if (Scalars.lessThan(den.abs(), RealScalar.of(1e-5))) {
      System.out.println("denominator den = " + den);
    }
    final Scalar factor = params.gForce().divide(den); // explain why no risk to divide by 0?
    Scalar Fz1L = Total.of(Tensors.of( //
        FB.dot(Cross2D.of(GC)), //
        FB.dot(Cross2D.of(HD)), //
        HD.dot(Cross2D.of(GC)) //
    )).multiply(factor).Get();
    Scalar Fz1R = Total.of(Tensors.of( //
        GC.dot(Cross2D.of(EA)), //
        HD.dot(Cross2D.of(EA)), //
        HD.dot(Cross2D.of(GC)) //
    )).multiply(factor).Get();
    Scalar Fz2L = Total.of(Tensors.of( //
        EA.dot(Cross2D.of(FB)), //
        EA.dot(Cross2D.of(HD)), //
        FB.dot(Cross2D.of(HD)) //
    )).multiply(factor).Get();
    Scalar Fz2R = Total.of(Tensors.of( //
        EA.dot(Cross2D.of(FB)), //
        GC.dot(Cross2D.of(EA)), //
        GC.dot(Cross2D.of(FB)) //
    )).multiply(factor).Get();
    // if (false) {
    // Scalar lF = params.levers().Get(0, 0);
    // Scalar lR = params.levers().Get(2, 0).negate();
    // Scalar lR_lF = lF.add(lR).multiply(RealScalar.of(2));
    // Fz1L = lR.divide(lR_lF).multiply(params.gForce());
    // Fz1R = lR.divide(lR_lF).multiply(params.gForce());
    // Fz2L = lF.divide(lR_lF).multiply(params.gForce());
    // Fz2R = lF.divide(lR_lF).multiply(params.gForce());
    // }
    Tensor fbodyZ = Tensors.of(Fz1L, Fz1R, Fz2L, Fz2R);
    //
    fwheel = Tensors.of( //
        mu1L.slip().multiply(Fz1L), //
        mu1R.slip().multiply(Fz1R), //
        mu2L.slip().multiply(Fz2L), //
        mu2R.slip().multiply(Fz2R) //
    ).unmodifiable();
    Tensor fbody = Tensors.empty();
    for (int index = 0; index < 4; ++index) {
      final Tensor _Fxy = RotationMatrix.of(angles.Get(index)).dot(fwheel.get(index)); // wheel to body
      fbody.append(Tensors.of(_Fxy.Get(0), _Fxy.Get(1), fbodyZ.Get(index)));
    }
    Forces = fbody.unmodifiable();
  }

  /** @param delta angle of wheel
   * @param index of wheel
   * @return */
  private Tensor get_ui_2d(Scalar delta, int index) { // as in doc
    Tensor tangent_2 = cs.u_2d().add(Cross2D.of(params.levers().get(index).extract(0, 2).multiply(cs.r)));
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
    Tensor tangent_3 = cs.u_3d().add(Cross.of(cs.rate_3d(), params.levers().get(index)));
    return rotation_3.dot(tangent_3).extract(0, 2);
  }

  /***************************************************/
  // FOR TESTS ONLY
  Tensor asVectorFX() { // Tensors.of(Fx1L, Fx1R, Fx2L, Fx2R);
    return Transpose.of(Forces).get(0);
  }

  Tensor asVectorFY() { // Tensors.of(Fy1L, Fy1R, Fy2L, Fy2R);
    return Transpose.of(Forces).get(1);
  }

  Tensor asVectorFZ() { // Tensors.of(Fz1L, Fz1R, Fz2L, Fz2R);
    return Transpose.of(Forces).get(2);
  }

  Tensor asVector_fX() { // Tensors.of(fx1L, fx1R, fx2L, fx2R);
    return Transpose.of(fwheel).get(0);
  }

  Tensor asVector_fY() { // Tensors.of(fy1L, fy1R, fy2L, fy2R);
    return Transpose.of(fwheel).get(1);
  }
}
