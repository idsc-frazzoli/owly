// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.Collection;

import ch.ethz.idsc.owly.math.RotationUtils;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class Se2ControlsTest extends TestCase {
  public void testSimple() {
    Collection<Flow> controls = Se2Controls.createControls(RotationUtils.DEGREE(45), 6);
    Scalar maxSpeed = Se2Controls.maxSpeed(controls);
    assertTrue(Chop._13.close(maxSpeed, RealScalar.ONE));
    Scalar maxTurn = Se2Controls.maxTurning(controls);
    assertTrue(Chop._13.close(maxTurn, RealScalar.of(45 * Math.PI / 180)));
  }
}
