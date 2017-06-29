// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;
import junit.framework.TestCase;

public class Normalize2DTest extends TestCase {
  public void testUp() {
    double eps = Math.nextUp(0);
    assertFalse(Normalize2D.unlessZero(RealScalar.of(eps), RealScalar.ZERO).equals(Array.zeros(2)));
    assertFalse(Normalize2D.unlessZero(RealScalar.ZERO, RealScalar.of(eps)).equals(Array.zeros(2)));
  }

  public void testDown() {
    double eps = Math.nextDown(0);
    assertFalse(Normalize2D.unlessZero(RealScalar.of(eps), RealScalar.ZERO).equals(Array.zeros(2)));
    assertFalse(Normalize2D.unlessZero(RealScalar.ZERO, RealScalar.of(eps)).equals(Array.zeros(2)));
  }

  public void testZero() {
    Tensor res = Normalize2D.unlessZero(RealScalar.ZERO, RealScalar.ZERO);
    assertEquals(res, Array.zeros(2));
  }
}
