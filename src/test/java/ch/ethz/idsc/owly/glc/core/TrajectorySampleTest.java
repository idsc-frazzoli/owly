// code by jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class TrajectorySampleTest extends TestCase {
  public void testHead() {
    TrajectorySample ts = TrajectorySample.head(new StateTime(Tensors.vector(2, 3), RealScalar.ZERO));
    assertFalse(ts.getFlow().isPresent());
    assertFalse(ts.toInfoString().isEmpty());
  }

  public void testFlow() {
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    Flow flow = StateSpaceModels.createFlow(stateSpaceModel, Tensors.vector(1, 1));
    TrajectorySample ts = new TrajectorySample(new StateTime(Tensors.vector(2, 3), RealScalar.ZERO), flow);
    assertTrue(ts.getFlow().isPresent());
    assertFalse(ts.toInfoString().isEmpty());
  }
}
