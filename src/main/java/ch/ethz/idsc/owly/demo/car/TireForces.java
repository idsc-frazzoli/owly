// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Hypot;
import ch.ethz.idsc.tensor.red.Total;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public class TireForces {
  //
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
  final Scalar fx1L; // 1
  final Scalar fx1R; // 2
  final Scalar fx2L; // 3
  final Scalar fx2R; // 4
  final Scalar fy1L; // 5
  final Scalar fy1R; // 6
  final Scalar fy2L; // 7
  final Scalar fy2R; // 8

  public TireForces(CarModel params, CarState cs, CarControl cc) {
    Scalar Ux1L = cs.getUx1L(cc.delta);
    Scalar Uy1L = cs.getUy1L(cc.delta);
    //
    Scalar Ux1R = cs.getUx1R(cc.delta);
    Scalar Uy1R = cs.getUy1R(cc.delta);
    //
    Scalar Ux2L = cs.getUx2L();
    Scalar Uy2L = cs.getUy2L();
    //
    Scalar Ux2R = cs.getUx2R();
    Scalar Uy2R = cs.getUy2R();
    //
    Scalar Sx1L = Ux1L.subtract(params.radiusTimes(cs.w1L)) //
        .divide(params.radiusTimes(cs.w1L));
    Scalar Sy1L = RealScalar.ONE.add(Sx1L).multiply(Uy1L.divide(Ux1L));
    //
    Scalar Sx1R = Ux1R.subtract(params.radiusTimes(cs.w1R)) //
        .divide(params.radiusTimes(cs.w1R));
    Scalar Sy1R = RealScalar.ONE.add(Sx1R).multiply(Uy1R.divide(Ux1R));
    //
    Scalar Sx2L = Ux2L.subtract(params.radiusTimes(cs.w2L)) //
        .divide(params.radiusTimes(cs.w2L));
    Scalar Sy2L = RealScalar.ONE.add(Sx2L).multiply(Uy2L.divide(Ux2L));
    //
    Scalar Sx2R = Ux2R.subtract(params.radiusTimes(cs.w2R)) //
        .divide(params.radiusTimes(cs.w2R));
    Scalar Sy2R = RealScalar.ONE.add(Sx2R).multiply(Uy2R.divide(Ux2R));
    //
    // System.out.println(Sx1L + " " + Sy1L);
    Scalar S1L = Hypot.bifunction.apply(Sx1L, Sy1L);
    // System.out.println(Sx1R + " " + Sy1R);
    Scalar S1R = Hypot.bifunction.apply(Sx1R, Sy1R);
    // System.out.println(Sx2L + " " + Sy2L);
    Scalar S2L = Hypot.bifunction.apply(Sx2L, Sy2L);
    // System.out.println(Sx2R + " " + Sy2R);
    Scalar S2R = Hypot.bifunction.apply(Sx2R, Sy2R);
    //
    // System.out.println("PACEJKA " + S1L);
    Scalar mu1L = params.pacejka1().apply(S1L);
    Scalar mu1R = params.pacejka1().apply(S1R);
    Scalar mu2L = params.pacejka2().apply(S2L);
    Scalar mu2R = params.pacejka2().apply(S2R);
    // ---
    // TODO investigate numerics
    Scalar eps = RealScalar.of(1e-4);
    //
    Scalar mux1L = mu1L.multiply(robustDiv(Sx1L, S1L, eps)).negate();
    Scalar muy1L = mu1L.multiply(robustDiv(Sy1L, S1L, eps)).negate();
    //
    Scalar mux1R = mu1R.multiply(robustDiv(Sx1R, S1R, eps)).negate();
    Scalar muy1R = mu1R.multiply(robustDiv(Sy1R, S1R, eps)).negate();
    //
    Scalar mux2L = mu2L.multiply(robustDiv(Sx2L, S2L, eps)).negate();
    Scalar muy2L = mu2L.multiply(robustDiv(Sy2L, S2L, eps)).negate();
    //
    Scalar mux2R = mu2R.multiply(robustDiv(Sx2R, S2R, eps)).negate();
    Scalar muy2R = mu2R.multiply(robustDiv(Sy2R, S2R, eps)).negate();
    //
    Scalar C1 = Total.prod(Tensors.of( //
        params.mu(), mux1L, params.heightCog(), Sin.of(cc.delta))).Get().negate();
    Scalar C2 = Total.prod(Tensors.of( //
        params.mu(), muy1L, params.heightCog(), Cos.of(cc.delta))).Get().negate();
    //
    Scalar C3 = Total.prod(Tensors.of( //
        params.mu(), mux1R, params.heightCog(), Sin.of(cc.delta))).Get().negate();
    Scalar C4 = Total.prod(Tensors.of( //
        params.mu(), muy1R, params.heightCog(), Cos.of(cc.delta))).Get().negate();
    //
    Scalar C5 = Total.prod(Tensors.of( //
        params.mu(), muy2L, params.heightCog())).Get().negate();
    Scalar C6 = Total.prod(Tensors.of( //
        params.mu(), muy2R, params.heightCog())).Get().negate();
    //
    Scalar K1 = Total.prod(Tensors.of( //
        params.mu(), mux1L, params.heightCog(), Cos.of(cc.delta))).Get();
    Scalar K2 = Total.prod(Tensors.of( //
        params.mu(), muy1L, params.heightCog(), Sin.of(cc.delta))).Get();
    //
    Scalar K3 = Total.prod(Tensors.of( //
        params.mu(), mux1R, params.heightCog(), Cos.of(cc.delta))).Get();
    Scalar K4 = Total.prod(Tensors.of( //
        params.mu(), muy1R, params.heightCog(), Sin.of(cc.delta))).Get();
    //
    Scalar K5 = Total.prod(Tensors.of( //
        params.mu(), mux2L, params.heightCog())).Get();
    Scalar K6 = Total.prod(Tensors.of( //
        params.mu(), mux2R, params.heightCog())).Get();
    //
    Scalar A = params.lw().negate().subtract(C1).subtract(C2);
    Scalar B = params.lw().subtract(C3).subtract(C4);
    Scalar C = params.lw().negate().subtract(C5);
    Scalar D = params.lw().subtract(C6);
    Scalar E = K1.subtract(K2).subtract(params.lF());
    Scalar F = K3.subtract(K4).subtract(params.lF());
    Scalar G = K5.add(params.lR());
    Scalar H = K6.add(params.lR());
    // ---
    Scalar den;
    {
      Tensor vec1 = Tensors.of(A, B, A, C, B, D, C, D);
      Tensor vec2 = Tensors.of( //
          F, E.negate(), G.negate(), E, H, F.negate(), H.negate(), G);
      den = vec1.dot(vec2).Get().multiply(RealScalar.of(2));
    }
    //
    final Scalar factor = params.gForce().divide(den);
    // final Scalar Fz1L;
    {
      Tensor vec1 = Tensors.of(B, C, B, D, C, D);
      Tensor vec2 = Tensors.of( //
          G, F.negate(), H, F.negate(), H.negate(), G);
      Fz1L = vec1.dot(vec2).Get().multiply(factor);
    }
    // final Scalar Fz1R;
    {
      Tensor vec1 = Tensors.of(A, C, A, D, C, D);
      Tensor vec2 = Tensors.of( //
          G, E.negate(), H, E.negate(), H, G.negate());
      Fz1R = vec1.dot(vec2).Get().multiply(factor).negate();
    }
    // final Scalar Fz2L;
    {
      Tensor vec1 = Tensors.of(A, B, A, D, B, D);
      Tensor vec2 = Tensors.of( //
          F, E.negate(), H, E.negate(), H, F.negate());
      Fz2L = vec1.dot(vec2).Get().multiply(factor);
    }
    // final Scalar Fz2R;
    {
      Tensor vec1 = Tensors.of(A, B, A, C, B, C);
      Tensor vec2 = Tensors.of( //
          F, E.negate(), G.negate(), E, G.negate(), F);
      Fz2R = vec1.dot(vec2).Get().multiply(factor);
    }
    //
    // Scalar
    fx1L = params.mu().multiply(Fz1L).multiply(mux1L);
    // Scalar
    fy1L = params.mu().multiply(Fz1L).multiply(muy1L);
    //
    // Scalar
    fx1R = params.mu().multiply(Fz1R).multiply(mux1R);
    // Scalar
    fy1R = params.mu().multiply(Fz1R).multiply(muy1R);
    //
    // Scalar
    fx2L = params.mu().multiply(Fz2L).multiply(mux2L);
    // Scalar
    fy2L = params.mu().multiply(Fz2L).multiply(muy2L);
    //
    // Scalar
    fx2R = params.mu().multiply(Fz2R).multiply(mux2R);
    // Scalar
    fy2R = params.mu().multiply(Fz2R).multiply(muy2R);
    //
    // TODO matrix mult
    Fx1L = fx1L.multiply(Cos.of(cc.delta)).subtract(fy1L.multiply(Sin.of(cc.delta)));
    Fy1L = fx1L.multiply(Sin.of(cc.delta)).add(fy1L.multiply(Cos.of(cc.delta)));
    //
    Fx1R = fx1R.multiply(Cos.of(cc.delta)).subtract(fy1R.multiply(Sin.of(cc.delta)));
    Fy1R = fx1R.multiply(Sin.of(cc.delta)).add(fy1R.multiply(Cos.of(cc.delta)));
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

  public static Scalar robustDiv(Scalar num, Scalar den, Scalar eps) {
    if (Scalars.isZero(den)) {
      if (Scalars.nonZero(num))
        return num.divide(eps);
      return RealScalar.ZERO;
    }
    return num.divide(den);
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
