// code by jph
package ch.ethz.idsc.owly.demo.rrts.rn;

import java.util.List;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class RnTransitionTest extends TestCase {
  public void testSimple() {
    RnTransition rnt = new RnTransition(Tensors.vector(2, 0), Tensors.vector(10, 0));
    List<StateTime> list = rnt.sampled(RealScalar.of(100), RealScalar.of(0), RealScalar.of(1));
    list.stream().map(StateTime::info).forEach(System.out::println);
  }
}
