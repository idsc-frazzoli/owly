// code by jph 
package ch.ethz.idsc.owly.demo.rn;

import ch.ethz.idsc.owly.math.region.TensorRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class RnPointcloudRegionTest extends TestCase {
  public void testEmpty() {
    TensorRegion rn = RnPointcloudRegion.of(Tensors.empty(), RealScalar.ONE);
    assertFalse(rn.isMember(Tensors.vector(2, 2.5)));
    assertFalse(rn.isMember(Tensors.vector(2, 2)));
    assertFalse(rn.isMember(Tensors.vector(2, 1)));
    assertFalse(rn.isMember(Tensors.vector(7, 1)));
  }

  public void testSingle2D() {
    TensorRegion rn = RnPointcloudRegion.of(Tensors.matrix(new Number[][] { { 2, 3 } }), RealScalar.ONE);
    assertTrue(rn.isMember(Tensors.vector(2, 2.5)));
    assertTrue(rn.isMember(Tensors.vector(2, 2)));
    assertFalse(rn.isMember(Tensors.vector(2, 1)));
    assertFalse(rn.isMember(Tensors.vector(7, 1)));
  }

  public void testTwo2D() {
    TensorRegion rn = RnPointcloudRegion.of(Tensors.matrix(new Number[][] { //
        { 2, 3 }, //
        { 7, 1 } //
    }), RealScalar.ONE);
    assertTrue(rn.isMember(Tensors.vector(2, 2.5)));
    assertTrue(rn.isMember(Tensors.vector(2, 2)));
    assertFalse(rn.isMember(Tensors.vector(2, 1)));
    assertTrue(rn.isMember(Tensors.vector(7, 1)));
    assertTrue(rn.isMember(Tensors.vector(7, 2)));
    assertFalse(rn.isMember(Tensors.vector(8, 2)));
  }
}
