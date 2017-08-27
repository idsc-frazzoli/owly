// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.VectorQ;
import ch.ethz.idsc.tensor.pdf.Distribution;
import ch.ethz.idsc.tensor.pdf.RandomVariate;
import ch.ethz.idsc.tensor.pdf.UniformDistribution;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Power;
import ch.ethz.idsc.tensor.sca.Sin;

public class CircleSampler {
  private static final Distribution POLAR = UniformDistribution.of(0, Math.PI * 2);
  private final Tensor center;
  private final Scalar radius;

  public CircleSampler(Tensor center, Scalar radius) {
    GlobalAssert.that(VectorQ.ofLength(center, 2));
    this.center = center;
    this.radius = radius;
  }

  // TODO this formula is only an approximation of a uniform distribution
  public Tensor next() {
    Scalar uniform = RandomVariate.of(UniformDistribution.unit());
    Scalar residue = RealScalar.ONE.subtract(Power.of(uniform, 2));
    Scalar alpha = RandomVariate.of(POLAR);
    return center.add(Tensors.of(Cos.FUNCTION.apply(alpha), Sin.FUNCTION.apply(alpha)) //
        .multiply(radius.multiply(residue)));
  }
}
