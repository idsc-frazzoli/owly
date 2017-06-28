// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.mat.RotationMatrix;

/** implementation has been verified through several tests */
public class TireForces {
  // forces in car frame
  final Scalar Fx1L; // 1
  final Scalar Fx1R; // 2
  final Scalar Fx2L; // 3
  final Scalar Fx2R; // 4
  final Scalar Fy1L; // 5
  final Scalar Fy1R; // 6
  final Scalar Fy2L; // 7
  final Scalar Fy2R; // 8
  private final Scalar Fz1L; // 9
  private final Scalar Fz1R; // 10
  private final Scalar Fz2L; // 11
  private final Scalar Fz2R; // 12
  //
  // forces in wheel frame
  final Scalar fx1L; // 1
  final Scalar fx1R; // 2
  final Scalar fx2L; // 3
  final Scalar fx2R; // 4
  private final Scalar fy1L; // 5
  private final Scalar fy1R; // 6
  private final Scalar fy2L; // 7
  private final Scalar fy2R; // 8

  public TireForces(CarModel params, CarState cs, CarControl cc) {
    // TODO check with edo
    final Tensor angles = cc.tire_angles().unmodifiable(); // params
    //
    final Tensor _u1L = cs.get_ui_2d(angles.Get(0), 0);
    final SlipInterface mu1L = new ReducedSlip(params.pacejka1(), _u1L, params.radiusTimes(cs.w1L));
    final Scalar mux1L = mu1L.slip().Get(0);
    final Scalar muy1L = mu1L.slip().Get(1);
    //
    final Tensor _u1R = cs.get_ui_2d(angles.Get(1), 1);
    final SlipInterface mu1R = new ReducedSlip(params.pacejka1(), _u1R, params.radiusTimes(cs.w1R));
    final Scalar mux1R = mu1R.slip().Get(0);
    final Scalar muy1R = mu1R.slip().Get(1);
    //
    final Tensor _u2L = cs.get_ui_2d(angles.Get(2), 2);
    final SlipInterface mu2L = new ReducedSlip(params.pacejka2(), _u2L, params.radiusTimes(cs.w2L));
    final Scalar mux2L = mu2L.slip().Get(0);
    final Scalar muy2L = mu2L.slip().Get(1);
    //
    final Tensor _u2R = cs.get_ui_2d(angles.Get(3), 3);
    final SlipInterface mu2R = new ReducedSlip(params.pacejka2(), _u2R, params.radiusTimes(cs.w2R));
    final Scalar mux2R = mu2R.slip().Get(0);
    final Scalar muy2R = mu2R.slip().Get(1);
    // ---
    final Scalar factor2 = params.mu().multiply(params.heightCog());
    final Tensor ck1L = RotationMatrix.of(angles.Get(0)).dot(mu1L.slip()).multiply(factor2);
    final Tensor ck1R = RotationMatrix.of(angles.Get(1)).dot(mu1R.slip()).multiply(factor2);
    final Tensor ck2L = RotationMatrix.of(angles.Get(2)).dot(mu2L.slip()).multiply(factor2);
    final Tensor ck2R = RotationMatrix.of(angles.Get(3)).dot(mu2R.slip()).multiply(factor2);
    // ---
    Scalar E = ck1L.Get(0).add(params.lF().negate()); // as in doc
    Scalar F = ck1R.Get(0).add(params.lF().negate()); // as in doc
    Scalar G = ck2L.Get(0).add(params.lR()); // as in doc
    Scalar H = ck2R.Get(0).add(params.lR()); // as in doc
    // ---
    Scalar A = ck1L.Get(1).add(params.lw().negate()); // as in doc
    Scalar B = ck1R.Get(1).add(params.lw()); // as in doc
    Scalar C = ck2L.Get(1).add(params.lw().negate()); // as in doc
    Scalar D = ck2R.Get(1).add(params.lw()); // as in doc
    // ---
    final Scalar den;
    {
      Tensor vec1 = Tensors.of(A, B, A, C, B, D, C, D);
      Tensor vec2 = Tensors.of( //
          F, E.negate(), G.negate(), E, H, F.negate(), H.negate(), G);
      den = vec1.dot(vec2).Get().multiply(RealScalar.of(2)); // as in doc
    }
    //
    if (Scalars.lessThan(den.abs(), RealScalar.of(1e-5))) {
      System.out.println("denominator den = " + den);
    }
    final Scalar factor = params.gForce().divide(den); // explain why no risk to divide by 0?
    {
      Tensor vec1 = Tensors.of(B, C, B, D, C, D);
      Tensor vec2 = Tensors.of( //
          G, F.negate(), H, F.negate(), H.negate(), G);
      Fz1L = vec1.dot(vec2).Get().multiply(factor);
    }
    {
      Tensor vec1 = Tensors.of(A, C, A, D, C, D);
      Tensor vec2 = Tensors.of( //
          G, E.negate(), H, E.negate(), H, G.negate());
      Fz1R = vec1.dot(vec2).Get().multiply(factor).negate();
    }
    {
      Tensor vec1 = Tensors.of(A, B, A, D, B, D);
      Tensor vec2 = Tensors.of( //
          F, E.negate(), H, E.negate(), H, F.negate());
      Fz2L = vec1.dot(vec2).Get().multiply(factor);
    }
    {
      Tensor vec1 = Tensors.of(A, B, A, C, B, C);
      Tensor vec2 = Tensors.of( //
          F, E.negate(), G.negate(), E, G.negate(), F);
      Fz2R = vec1.dot(vec2).Get().multiply(factor);
    }
    //
    fx1L = Fz1L.multiply(mux1L.multiply(params.mu()));
    fy1L = Fz1L.multiply(muy1L.multiply(params.mu()));
    //
    fx1R = Fz1R.multiply(mux1R.multiply(params.mu()));
    fy1R = Fz1R.multiply(muy1R.multiply(params.mu()));
    //
    fx2L = Fz2L.multiply(mux2L.multiply(params.mu()));
    fy2L = Fz2L.multiply(muy2L.multiply(params.mu()));
    //
    fx2R = Fz2R.multiply(mux2R.multiply(params.mu()));
    fy2R = Fz2R.multiply(muy2R.multiply(params.mu()));
    //
    final Tensor _F1L = RotationMatrix.of(angles.Get(0)).dot(Tensors.of(fx1L, fy1L)); // wheel to body
    Fx1L = _F1L.Get(0);
    Fy1L = _F1L.Get(1);
    //
    final Tensor _F1R = RotationMatrix.of(angles.Get(1)).dot(Tensors.of(fx1R, fy1R)); // wheel to body
    Fx1R = _F1R.Get(0);
    Fy1R = _F1R.Get(1);
    //
    Fx2L = fx2L;
    Fy2L = fy2L;
    //
    Fx2R = fx2R;
    Fy2R = fy2R;
  }

  public Scalar total1234() {
    return Fx1L.add(Fx1R).add(Fx2L).add(Fx2R);
  }

  public Scalar total5678() {
    return Fy1L.add(Fy1R).add(Fy2L).add(Fy2R);
  }

  public Scalar total24_13() {
    return Fx1R.add(Fx2R).subtract(Fx1L.add(Fx2L));
  }

  public Scalar total56() {
    return Fy1L.add(Fy1R); // 5 + 6
  }

  public Scalar total78() {
    return Fy2L.add(Fy2R); // 7 + 8
  }

  /***************************************************/
  Tensor asVectorFX() { // only used in test
    return Tensors.of(Fx1L, Fx1R, Fx2L, Fx2R);
  }

  Tensor asVectorFY() { // only used in test
    return Tensors.of(Fy1L, Fy1R, Fy2L, Fy2R);
  }

  Tensor asVectorFZ() { // only used in test
    return Tensors.of(Fz1L, Fz1R, Fz2L, Fz2R);
  }

  Tensor asVector_fX() { // only used in test
    return Tensors.of(fx1L, fx1R, fx2L, fx2R);
  }

  Tensor asVector_fY() { // only used in test
    return Tensors.of(fy1L, fy1R, fy2L, fy2R);
  }
}
