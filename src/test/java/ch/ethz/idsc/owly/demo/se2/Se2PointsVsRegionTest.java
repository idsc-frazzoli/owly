// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class Se2PointsVsRegionTest extends TestCase {
  public void testSimple() {
    Se2PointsVsRegion r = new Se2PointsVsRegion(Tensors.matrixInt(new int[][] { { 3, 4 }, { 5, 6 } }), null);
    assertEquals(r.points().get(Tensor.ALL, 2), Tensors.vector(1, 1));
  }
}
