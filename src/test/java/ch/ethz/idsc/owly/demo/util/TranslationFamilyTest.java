// code by jph
package ch.ethz.idsc.owly.demo.util;

import ch.ethz.idsc.owly.math.se2.BijectionFamily;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.pdf.DiscreteUniformDistribution;
import ch.ethz.idsc.tensor.pdf.Distribution;
import ch.ethz.idsc.tensor.pdf.RandomVariate;
import junit.framework.TestCase;

public class TranslationFamilyTest extends TestCase {
  public void testSimple() {
    BijectionFamily bijectionFamily = new SimpleTranslationFamily(s -> Tensors.of(RealScalar.of(3), RealScalar.of(100).add(s)));
    Distribution distribution = DiscreteUniformDistribution.of(-15, 16);
    for (int index = 0; index < 100; ++index) {
      Scalar scalar = RandomVariate.of(distribution);
      Tensor point = RandomVariate.of(distribution, 2);
      Tensor fwd = bijectionFamily.forward(scalar).apply(point);
      assertEquals(bijectionFamily.inverse(scalar).apply(fwd), point);
    }
  }
}
