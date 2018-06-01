package ch.ethz.idsc.owl.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class VectorScalarTest extends TestCase {
  public void testOne() {
    Scalar a = VectorScalar.of(Tensors.vector(1, -1, 2));
    assertEquals(a.abs(), VectorScalar.of(Tensors.vector(1, 1, 2)));
    assertEquals(a.add(VectorScalar.of(Tensors.vector(0, 1, 0))), VectorScalar.of(Tensors.vector(1, 0, 2)));
    assertEquals(a.divide(RealScalar.of(2)), VectorScalar.of(Tensors.vector(0.5, -0.5, 1)));
    assertEquals(a.reciprocal(), VectorScalar.of(Tensors.vector(1, -1, 0.5)));
    assertEquals(a.zero(), VectorScalar.of(Tensors.vector(0, 0, 0)));
    assertEquals(((VectorScalar) a).vector().length(), 3);
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
