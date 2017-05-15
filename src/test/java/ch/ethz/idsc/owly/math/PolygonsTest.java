// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class PolygonsTest extends TestCase {
  public void testInside() {
    Tensor polygon = Tensors.matrix(new Number[][] { //
        { 0, 0 }, //
        { 1, 0 }, //
        { 1, 1 }, //
        { 0, 1 } //
    });
    assertTrue(Polygons.isInside(polygon, Tensors.vector(.5, .5)));
    assertTrue(Polygons.isInside(polygon, Tensors.vector(.9, .9)));
    assertTrue(Polygons.isInside(polygon, Tensors.vector(.1, .1)));
    assertFalse(Polygons.isInside(polygon, Tensors.vector(.1, -.1)));
    assertFalse(Polygons.isInside(polygon, Tensors.vector(1, 1.1)));
  }
}
