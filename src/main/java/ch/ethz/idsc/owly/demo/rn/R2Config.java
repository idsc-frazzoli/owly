// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Range;
import ch.ethz.idsc.tensor.lie.AngleVector;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Sign;

public class R2Config {
  private final Scalar speed;

  public R2Config(Scalar speed) {
    GlobalAssert.that(Sign.isPositive(speed));
    this.speed = speed;
  }

  public Collection<Flow> createRadial(final int num) {
    GlobalAssert.that(2 < num); // otherwise does not cover plane
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Range.of(0, num).multiply(DoubleScalar.of(2 * Math.PI / num))) {
      Tensor u = Chop._10.of(AngleVector.of(angle.Get()).multiply(speed));
      list.add(StateSpaceModels.createFlow(stateSpaceModel, u));
    }
    return list;
  }

  public Flow stayPut(int u_length) {
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    return StateSpaceModels.createFlow(stateSpaceModel, Array.zeros(u_length));
  }
}
