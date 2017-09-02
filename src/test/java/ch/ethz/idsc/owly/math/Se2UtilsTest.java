// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.mat.Det;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class Se2UtilsTest extends TestCase {
  public void testSimple1() {
    Tensor matrix = Se2Utils.toSE2Matrix(Tensors.vector(2, 3, 4));
    assertEquals(matrix.get(2), Tensors.vector(0, 0, 1));
    Scalar det = Det.of(matrix);
    assertTrue(Chop._14.close(det, RealScalar.ONE));
  }

  public void testSimple2() {
    Tensor matrix = Se2Utils.toSE2MatrixTranspose(Tensors.vector(2, 3, 4));
    assertEquals(matrix.get(Tensor.ALL, 2), Tensors.vector(0, 0, 1));
    Scalar det = Det.of(matrix);
    assertTrue(Chop._14.close(det, RealScalar.ONE));
  }
}
