// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.mat.RotationMatrix;
import ch.ethz.idsc.tensor.red.Total;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

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
  final Scalar Fz1L; // 9
  final Scalar Fz1R; // 10
  final Scalar Fz2L; // 11
  final Scalar Fz2R; // 12
  //
  // forces in wheel frame
  final Scalar fx1L; // 1
  final Scalar fx1R; // 2
  final Scalar fx2L; // 3
  final Scalar fx2R; // 4
  final Scalar fy1L; // 5
  final Scalar fy1R; // 6
  final Scalar fy2L; // 7
  final Scalar fy2R; // 8

  public TireForces(CarModel params, CarState cs, CarControl cc) {
    // TODO check with edo
    final Tensor angles = cc.tire_angles().unmodifiable(); // params
    //
    final Tensor _u1L = cs.get_ui_2d(angles.Get(0), 0);
    final SlipInterface sr1L = new ReducedSlip(params.pacejka1(), _u1L, params.radiusTimes(cs.w1L));
    final Scalar mux1L = sr1L.slip().Get(0);
    final Scalar muy1L = sr1L.slip().Get(1);
    //
    final Tensor _u1R = cs.get_ui_2d(angles.Get(1), 1);
    final SlipInterface sr1R = new ReducedSlip(params.pacejka1(), _u1R, params.radiusTimes(cs.w1R));
    final Scalar mux1R = sr1R.slip().Get(0);
    final Scalar muy1R = sr1R.slip().Get(1);
    //
    final Tensor _u2L = cs.get_ui_2d(angles.Get(2), 2);
    final SlipInterface sr2L = new ReducedSlip(params.pacejka2(), _u2L, params.radiusTimes(cs.w2L));
    final Scalar mux2L = sr2L.slip().Get(0);
    final Scalar muy2L = sr2L.slip().Get(1);
    //
    final Tensor _u2R = cs.get_ui_2d(angles.Get(3), 3);
    final SlipInterface sr2R = new ReducedSlip(params.pacejka2(), _u2R, params.radiusTimes(cs.w2R));
    final Scalar mux2R = sr2R.slip().Get(0);
    final Scalar muy2R = sr2R.slip().Get(1);
    // ---
    // related to 1L
    Scalar C1 = Total.prod(Tensors.of( //
        params.mu(), mux1L, params.heightCog(), Sin.of(angles.Get(0)))).Get().negate();
    Scalar C2 = Total.prod(Tensors.of( //
        params.mu(), muy1L, params.heightCog(), Cos.of(angles.Get(0)))).Get().negate();
    //
    // related to 1R
    Scalar C3 = Total.prod(Tensors.of( //
        params.mu(), mux1R, params.heightCog(), Sin.of(angles.Get(1)))).Get().negate();
    Scalar C4 = Total.prod(Tensors.of( //
        params.mu(), muy1R, params.heightCog(), Cos.of(angles.Get(1)))).Get().negate();
    //
    Scalar C5 = Total.prod(Tensors.of( //
        params.mu(), muy2L, params.heightCog())).Get().negate();
    Scalar C6 = Total.prod(Tensors.of( //
        params.mu(), muy2R, params.heightCog())).Get().negate();
    //
    // related to 1L
    Scalar K1 = Total.prod(Tensors.of( //
        params.mu(), mux1L, params.heightCog(), Cos.of(angles.Get(0)))).Get();
    Scalar K2 = Total.prod(Tensors.of( //
        params.mu(), muy1L, params.heightCog(), Sin.of(angles.Get(0)))).Get();
    //
    // related to 1R
    Scalar K3 = Total.prod(Tensors.of( //
        params.mu(), mux1R, params.heightCog(), Cos.of(angles.Get(1)))).Get();
    Scalar K4 = Total.prod(Tensors.of( //
        params.mu(), muy1R, params.heightCog(), Sin.of(angles.Get(1)))).Get();
    //
    Scalar K5 = Total.prod(Tensors.of( //
        params.mu(), mux2L, params.heightCog())).Get();
    Scalar K6 = Total.prod(Tensors.of( //
        params.mu(), mux2R, params.heightCog())).Get();
    //
    Scalar A = params.lw().negate().subtract(C1).subtract(C2); // as in doc
    Scalar B = params.lw().subtract(C3).subtract(C4); // as in doc
    Scalar C = params.lw().negate().subtract(C5); // as in doc
    Scalar D = params.lw().subtract(C6); // as in doc
    Scalar E = K1.subtract(K2).subtract(params.lF()); // as in doc
    Scalar F = K3.subtract(K4).subtract(params.lF()); // as in doc
    Scalar G = K5.add(params.lR()); // as in doc
    Scalar H = K6.add(params.lR()); // as in doc
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
    fx1L = params.mu().multiply(Fz1L).multiply(mux1L);
    fy1L = params.mu().multiply(Fz1L).multiply(muy1L);
    //
    fx1R = params.mu().multiply(Fz1R).multiply(mux1R);
    fy1R = params.mu().multiply(Fz1R).multiply(muy1R);
    //
    fx2L = params.mu().multiply(Fz2L).multiply(mux2L);
    fy2L = params.mu().multiply(Fz2L).multiply(muy2L);
    //
    fx2R = params.mu().multiply(Fz2R).multiply(mux2R);
    fy2R = params.mu().multiply(Fz2R).multiply(muy2R);
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

  public Tensor asVectorFX() {
    return Tensors.of(Fx1L, Fx1R, Fx2L, Fx2R);
  }

  public Tensor asVectorFY() {
    return Tensors.of(Fy1L, Fy1R, Fy2L, Fy2R);
  }

  public Tensor asVectorFZ() {
    return Tensors.of(Fz1L, Fz1R, Fz2L, Fz2R);
  }

  public Tensor asVector_fX() {
    return Tensors.of(fx1L, fx1R, fx2L, fx2R);
  }

  public Tensor asVector_fY() {
    return Tensors.of(fy1L, fy1R, fy2L, fy2R);
  }
}
