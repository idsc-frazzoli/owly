// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import junit.framework.TestCase;

public class Se2StateSpaceModelTest extends TestCase {
  public void testSimple() {
    StateSpaceModel stateSpaceModel = Se2StateSpaceModel.INSTANCE;
    assertEquals(stateSpaceModel.getLipschitz(), RealScalar.ONE);
  }
}
