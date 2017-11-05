// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.math.RotationUtils;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class Se2ControlsTest extends TestCase {
  public void testSimple() {
    Collection<Flow> controls = Se2Controls.createControls(RotationUtils.DEGREE(45), 6);
    Scalar maxSpeed = Se2Controls.maxSpeed(controls);
    assertTrue(Chop._13.close(maxSpeed, RealScalar.ONE));
    Scalar maxTurn = Se2Controls.maxTurning(controls);
    assertTrue(Chop._13.close(maxTurn, RealScalar.of(45 * Math.PI / 180)));
    assertTrue(Chop._13.close(maxTurn, RotationUtils.DEGREE(45)));
  }

  public void testMaxRate() {
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Subdivide.of(RealScalar.of(-.1), RealScalar.of(.3), 5))
      list.add(Se2Controls.singleton(RealScalar.of(2), angle));
    Scalar maxR = Se2Controls.maxTurning(list);
    assertEquals(maxR, RealScalar.of(.6));
  }

  public void testMaxRate2() {
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Subdivide.of(RealScalar.of(-.3), RealScalar.of(.1), 5))
      list.add(Se2Controls.singleton(RealScalar.of(2), angle));
    Scalar maxR = Se2Controls.maxTurning(list);
    assertEquals(maxR, RealScalar.of(.6));
  }
}
