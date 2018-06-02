package ch.ethz.idsc.owl.math;

import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class VectorScalarTest extends TestCase {
  public void testOne() {
    Scalar a = VectorScalar.of(Tensors.vector(1, -1, 2));
    assertEquals(a.abs(), VectorScalar.of(Tensors.vector(1, 1, 2)));
    assertEquals(a.add(VectorScalar.of(Tensors.vector(0, 1, 0))), VectorScalar.of(Tensors.vector(1, 0, 2)));
    assertEquals(a.divide(RealScalar.of(2)), VectorScalar.of(Tensors.vector(0.5, -0.5, 1)));
    assertEquals(a.zero(), VectorScalar.of(Tensors.vector(0, 0, 0)));
    assertEquals(((VectorScalar) a).vector().length(), 3);
    // ---
    a = VectorScalar.of(Tensors.vector(0.00001, 0.00005, 0));
    assertEquals(((VectorScalar) a).chop(Chop._04), VectorScalar.of(Tensors.vector(0, 0, 0)));
    // ---
    a = VectorScalar.of(Tensors.of(RealScalar.ONE, DoubleScalar.NEGATIVE_INFINITY));
    assertFalse(((VectorScalar) a).isMachineNumber());
    assertFalse(((VectorScalar) a).isExactScalar());
    a = VectorScalar.of(Tensors.of(RealScalar.ONE, DoubleScalar.ONE));
    // assertTrue(((VectorScalar) a).isMachineNumber()); //FIXME
    assertTrue(((VectorScalar) a).isExactScalar());
    // ---
    Scalar v1 = VectorScalar.of(Tensors.vector(1, 6, 1));
    Scalar v2 = VectorScalar.of(Tensors.vector(1, 5, 10));
    Comparable<Scalar> comparable = (Comparable<Scalar>) v1;
    assertEquals(comparable.compareTo(v2), Integer.compare(1, 0));
  }

  public void testFail() {
    try {
      VectorScalar.of(Tensors.empty()).number();
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
    try {
      VectorScalar.of(Tensors.empty().add(RealScalar.ONE));
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
