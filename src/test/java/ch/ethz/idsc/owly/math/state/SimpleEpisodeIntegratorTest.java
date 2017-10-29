// code by jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;

import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.MidpointIntegrator;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.flow.RungeKutta4Integrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class SimpleEpisodeIntegratorTest extends TestCase {
  public void testSimple() {
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    Tensor u = Tensors.vector(5, -2);
    Tensor x = Tensors.vector(1, 2);
    Scalar t = RealScalar.of(3);
    Scalar p = RealScalar.of(2);
    Integrator[] ints = new Integrator[] { //
        EulerIntegrator.INSTANCE, //
        MidpointIntegrator.INSTANCE, //
        RungeKutta4Integrator.INSTANCE, //
        RungeKutta45Integrator.INSTANCE //
    };
    for (Integrator integrator : ints) {
      AbstractEpisodeIntegrator aei = new SimpleEpisodeIntegrator( //
          SingleIntegratorStateSpaceModel.INSTANCE, //
          integrator, new StateTime(x, t));
      Flow flow = StateSpaceModels.createFlow(stateSpaceModel, u);
      List<StateTime> list = aei.move(flow, p);
      assertEquals(list.size(), 1);
      Tensor cmp = x.add(u.multiply(p));
      // System.out.println(list.get(0).toInfoString());
      assertEquals(list.get(0).state(), cmp);
      assertEquals(list.get(0).time(), t.add(p));
    }
  }
}
