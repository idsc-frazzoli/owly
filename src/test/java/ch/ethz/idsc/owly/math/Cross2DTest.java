// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.mat.RotationMatrix;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class Cross2DTest extends TestCase {
  public void testSimple() {
    // Cross[{1, 2}] == {-2, 1}
    assertEquals(Cross2D.of(Tensors.vector(1, 2)), Tensors.vector(-2, 1));
  }

  public void testRotation() {
    Tensor x = Tensors.vector(1, 2);
    Tensor mat = RotationMatrix.of(RealScalar.of(Math.PI / 2));
    assertTrue(Chop._10.close(Cross2D.of(x), mat.dot(x)));
  }
}
