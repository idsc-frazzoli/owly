// code by jph
package ch.ethz.idsc.owly.demo.twd;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.qty.Unit;
import ch.ethz.idsc.tensor.qty.Units;
import ch.ethz.idsc.tensor.sca.Round;
import junit.framework.TestCase;

public class TwdConfigTest extends TestCase {
  public void testMaxSpeed() {
    TwdConfig twdConfig = new TwdConfig(RealScalar.of(3), RealScalar.of(0.567));
    Collection<Flow> controls = twdConfig.createControls(8);
    Scalar maxSpeed = Se2Controls.maxSpeed(controls);
    assertEquals(maxSpeed, RealScalar.of(3));
  }

  public void testUnit() {
    Scalar ms = Quantity.of(3, "m*s^-1");
    Scalar sa = Quantity.of(0.567, "m");
    TwdConfig twdConfig = new TwdConfig(ms, sa);
    Collection<Flow> controls = twdConfig.createControls(8);
    Scalar maxSpeed = Se2Controls.maxSpeed(controls);
    assertEquals(maxSpeed, ms);
    Scalar maxTurng = Se2Controls.maxTurning(controls);
    assertEquals(Units.of(maxTurng), Unit.of("s^-1"));
    assertEquals(maxTurng, ms.divide(sa));
  }

  public void testNoDuplicates() {
    Scalar ms = Quantity.of(3, "m*s^-1");
    Scalar sa = Quantity.of(0.567, "m");
    TwdConfig twdConfig = new TwdConfig(ms, sa);
    for (int res = 3; res <= 8; ++res) {
      Collection<Flow> controls = twdConfig.createControls(res);
      Set<Tensor> set = new HashSet<>();
      for (Flow flow : controls) {
        Tensor key = flow.getU().map(Round._3);
        assertTrue(set.add(key));
      }
    }
  }
}
