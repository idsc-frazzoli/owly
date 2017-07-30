// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class Normalize2DTest extends TestCase {
  public void testUp() {
    double eps = Math.nextUp(0.0);
    assertFalse(Normalize2D.unlessZero(RealScalar.of(eps), RealScalar.ZERO).equals(Array.zeros(2)));
    assertFalse(Normalize2D.unlessZero(RealScalar.ZERO, RealScalar.of(eps)).equals(Array.zeros(2)));
  }

  public void testDown() {
    double eps = Math.nextDown(0.0);
    Tensor vec = Normalize2D.unlessZero(RealScalar.of(eps), RealScalar.ZERO);
    assertEquals(vec, Tensors.vector(-1, 0));
    assertEquals(Normalize2D.unlessZero(RealScalar.ZERO, RealScalar.of(-eps)), Tensors.vector(0, 1));
  }

  public void testZero() {
    Tensor res = Normalize2D.unlessZero(RealScalar.ZERO, RealScalar.ZERO);
    assertEquals(res, Array.zeros(2));
  }

  public void testUp2() {
    double eps = Math.nextUp(0.0);
    Tensor vec = Normalize2D.unlessZero(RealScalar.of(eps), RealScalar.of(eps));
    assertTrue(Chop._12.close(Norm._2.of(vec), RealScalar.ONE));
  }

  public void testNaN() {
    try {
      Normalize2D.unlessZero(DoubleScalar.POSITIVE_INFINITY, RealScalar.ZERO);
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
    try {
      Normalize2D.unlessZero(DoubleScalar.INDETERMINATE, RealScalar.ZERO);
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
