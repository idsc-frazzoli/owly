// code by jph
package ch.ethz.idsc.owly.demo.rice;

import java.util.List;

import ch.ethz.idsc.owl.data.Lists;
import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.StateSpaceModels;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class Duncan1StateSpaceModelTest extends TestCase {
  public void testScalar() {
    StateSpaceModel stateSpaceModel = new Duncan1StateSpaceModel(Quantity.of(0.1, "s^-1"));
    Tensor x = Tensors.fromString("{10[m*s^-1], 20[m*s^-1]}");
    Tensor u = Tensors.fromString("{-1[m*s^-2], -1[m*s^-2]}");
    Tensor fxu = stateSpaceModel.f(x, u).multiply(Quantity.of(1, "s"));
    assertEquals(fxu, Tensors.fromString("{-2[m*s^-1], -3[m*s^-1]}"));
  }

  public void testZero() {
    StateSpaceModel stateSpaceModel = new Duncan1StateSpaceModel(Quantity.of(0.0, "s^-1"));
    Tensor x = Tensors.fromString("{10[m*s^-1], 20[m*s^-1]}");
    Tensor u = Tensors.fromString("{-1[m*s^-2], -1[m*s^-2]}");
    Tensor fxu = stateSpaceModel.f(x, u).multiply(Quantity.of(1, "s"));
    assertEquals(fxu, Tensors.fromString("{-1[m*s^-1], -1[m*s^-1]}"));
  }

  public void testLimit() {
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, Scalars.fromString("1/5[s]"), 99 * 5); // simulate for 100[s]
    StateTime stateTime = new StateTime(Tensors.of(Quantity.of(10, "m*s^-1")), Quantity.of(1, "s"));
    Scalar lambda = Quantity.of(2.0, "s^-1");
    StateSpaceModel stateSpaceModel = new Duncan1StateSpaceModel(lambda);
    Scalar push = Quantity.of(3, "m*s^-2");
    Flow flow = StateSpaceModels.createFlow(stateSpaceModel, Tensors.of(push));
    List<StateTime> list = stateIntegrator.trajectory(stateTime, flow);
    StateTime last = Lists.getLast(list);
    assertEquals(last.time(), Quantity.of(100, "s"));
    assertTrue(Chop._12.close(last.state().get(0), push.divide(lambda)));
  }

  public void testFail() {
    try {
      new Duncan1StateSpaceModel(Quantity.of(-1.0, "s^-1"));
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
