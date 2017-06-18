// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.tensor.RealScalar;
import junit.framework.TestCase;

public class Se2StateSpaceModelTest extends TestCase {
  public void testSimple() {
    Se2StateSpaceModel se2StateSpaceModel = new Se2StateSpaceModel();
    assertEquals(se2StateSpaceModel.getLipschitz(), RealScalar.ONE);
  }
}
