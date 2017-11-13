// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.se2.twd.TwdConfig;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.qty.Unit;
import ch.ethz.idsc.tensor.qty.Units;
import junit.framework.TestCase;

public class Se2LateralAccelerationTest extends TestCase {
  public void testCar() {
    final Scalar ms = Quantity.of(2, "m*s^-1");
    final Scalar mr = Scalars.fromString("3[rad*m^-1]");
    Flow flow = CarConfig.singleton(ms, mr);
    assertEquals(Units.of(flow.getU().Get(2)), Unit.of("rad*s^-1"));
    Tensor u = flow.getU();
    Scalar cost = Se2LateralAcceleration.cost(u, Quantity.of(3, "s"));
    assertEquals(Units.of(cost), Unit.of("rad^2*s^-1"));
  }

  public void testTwd() {
    Scalar ms = Quantity.of(3, "m*s^-1");
    Scalar sa = Quantity.of(0.567, "m*rad^-1");
    TwdConfig twdConfig = new TwdConfig(ms, sa);
    Collection<Flow> controls = twdConfig.createControls(8);
    Tensor u = controls.iterator().next().getU();
    Scalar cost = Se2LateralAcceleration.cost(u, Quantity.of(3, "s"));
    assertEquals(Units.of(cost), Unit.of("rad^2*s^-1"));
  }
}
