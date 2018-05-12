// code by jph
package ch.ethz.idsc.owl.math.region;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.opt.TensorScalarFunction;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class Cone2DistanceTest extends TestCase {
  public void testSimple() {
    TensorScalarFunction tsf = new Cone2Distance(Array.zeros(3), RealScalar.of(Math.PI / 4));
    assertEquals(tsf.apply(Tensors.vector(1, .1)), RealScalar.ZERO);
    assertEquals(tsf.apply(Tensors.vector(-4, 3)), RealScalar.of(5));
    assertEquals(tsf.apply(Tensors.vector(-4, -3)), RealScalar.of(5));
    assertTrue(Chop._06.close(tsf.apply(Tensors.vector(-1, 1)), RealScalar.of(Math.sqrt(2))));
    assertTrue(Chop._06.close(tsf.apply(Tensors.vector(-1, -1)), RealScalar.of(Math.sqrt(2))));
  }
}
