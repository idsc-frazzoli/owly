// code by jph
package ch.ethz.idsc.owl.math.planar;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class SignedCurvature2DTest extends TestCase {
  public void testCounterClockwise() {
    Tensor a = Tensors.vector(1, 0);
    Tensor b = Tensors.vector(0, 1);
    Tensor c = Tensors.vector(-1, 0);
    assertEquals(Chop._10.of(SignedCurvature2D.of(a, b, c).add(RealScalar.ONE)), RealScalar.ZERO);
    assertEquals(Chop._10.of(SignedCurvature2D.of(c, b, a).subtract(RealScalar.ONE)), RealScalar.ZERO);
  }

  public void testStraight() {
    Tensor a = Tensors.vector(1, 1);
    Tensor b = Tensors.vector(2, 2);
    Tensor c = Tensors.vector(5, 5);
    assertEquals(SignedCurvature2D.of(a, b, c), RealScalar.ZERO);
  }
}
