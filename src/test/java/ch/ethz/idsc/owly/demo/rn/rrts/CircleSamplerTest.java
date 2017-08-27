// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;
import junit.framework.TestCase;

public class CircleSamplerTest extends TestCase {
  public void testSimple() {
    CircleRandomSample circleSampler = new CircleRandomSample(Tensors.vector(0, 0), RealScalar.ONE);
    for (int c = 0; c < 100; ++c) {
      Tensor loc = circleSampler.nextSample();
      Scalar rad = Norm._2.of(loc);
      assertTrue(Scalars.lessThan(rad, RealScalar.ONE));
    }
  }
}
