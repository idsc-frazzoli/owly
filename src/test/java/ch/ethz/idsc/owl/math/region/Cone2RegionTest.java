// code by jph
package ch.ethz.idsc.owl.math.region;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class Cone2RegionTest extends TestCase {
  public void testSimple() {
    Cone2Region cone2Region = new Cone2Region(Tensors.vector(5, 0, Math.PI / 2), RealScalar.ONE);
    assertTrue(cone2Region.isMember(Tensors.vector(6, 2)));
    assertTrue(cone2Region.isMember(Tensors.vector(4, 2)));
    assertFalse(cone2Region.isMember(Tensors.vector(0, 2)));
    assertFalse(cone2Region.isMember(Tensors.vector(6, -2)));
  }

  public void test90deg() {
    Cone2Region cone2Region = new Cone2Region(Tensors.vector(5, 0, Math.PI / 2), RealScalar.of(Math.PI / 4));
    assertTrue(cone2Region.isMember(Tensors.vector(5 + 2, 2.1)));
    assertFalse(cone2Region.isMember(Tensors.vector(5 + 2, 1.9)));
  }

  public void testUnits() {
    Cone2Region cone2Region = new Cone2Region(Tensors.fromString("{3[m],4[m],1.5}"), RealScalar.of(1));
    assertTrue(cone2Region.isMember(Tensors.fromString("{3[m],4+1[m]}")));
    assertFalse(cone2Region.isMember(Tensors.fromString("{3[m],4-1[m]}")));
  }

  public void testNegativeFail() {
    try {
      new Cone2Region(Tensors.vector(5, 0, Math.PI / 2), RealScalar.of(-Math.PI));
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }

  public void testAngle() {
    try {
      new Cone2Region(Tensors.vector(5, 0, Math.PI / 2), RealScalar.of(Math.PI / 2 + 0.1));
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
