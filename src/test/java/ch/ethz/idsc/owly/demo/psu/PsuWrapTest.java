// code by jph
package ch.ethz.idsc.owly.demo.psu;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class PsuWrapTest extends TestCase {
  public void testPeriod() {
    assertTrue(Chop._12.allZero(PsuWrap.INSTANCE.distance(Tensors.vector(0, 0), Tensors.vector(2 * Math.PI, 0))));
    assertTrue(Chop._12.allZero(PsuWrap.INSTANCE.distance(Tensors.vector(0, 0), Tensors.vector(4 * Math.PI, 0))));
  }

  public void testLinear() {
    assertEquals(PsuWrap.INSTANCE.distance(Tensors.vector(0, 0), Tensors.vector(0, 10)), RealScalar.of(10));
    assertEquals(PsuWrap.INSTANCE.distance(Tensors.vector(0, -10), Tensors.vector(0, 10)), RealScalar.of(20));
  }
}
