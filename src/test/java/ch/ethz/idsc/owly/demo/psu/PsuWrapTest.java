// code by jph
package ch.ethz.idsc.owly.demo.psu;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class PsuWrapTest extends TestCase {
  public void testSimple() {
    Scalar d = PsuWrap.INSTANCE.distance(Tensors.vector(0, 0), Tensors.vector(2 * Math.PI, 0));
    assertTrue(Chop._12.allZero(d));
  }
}
