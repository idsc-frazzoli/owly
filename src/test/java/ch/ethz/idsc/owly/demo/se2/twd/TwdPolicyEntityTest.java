// code by jph
package ch.ethz.idsc.owly.demo.se2.twd;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class TwdPolicyEntityTest extends TestCase {
  public void testSimple() {
    CarPolicyEntity tpe = new CarPolicyEntity( //
        new StateTime(Tensors.vector(1, 2, 3), RealScalar.ZERO), null);
    assertEquals(tpe.actions(null).stream().distinct().count(), tpe.actions(null).length());
    // System.out.println(Pretty.of(tpe.actions()));
  }
}
