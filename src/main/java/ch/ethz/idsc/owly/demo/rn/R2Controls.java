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
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Chop;

// create radial controls
// class is intentionally public 
public enum R2Controls {
  ;
  // ---
  public static Collection<Flow> createRadial(final int num) {
    GlobalAssert.that(2 < num); // otherwise does not cover plane
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Range.of(0, num).multiply(DoubleScalar.of(2 * Math.PI / num))) {
      Tensor u = Chop._10.of(AngleVector.of(angle.Get()));
      list.add(StateSpaceModels.createFlow(stateSpaceModel, u));
    }
    return list;
  }

  public static Scalar maxSpeed(Collection<Flow> controls) {
    int length = controls.iterator().next().getU().length();
    GlobalAssert.that(length == 2);
    return controls.stream().map(Flow::getU).map(Norm._2::ofVector).reduce(Max::of).get();
  }

  // ---
  public static Flow stayPut(int u_length) {
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    return StateSpaceModels.createFlow(stateSpaceModel, Array.zeros(u_length));
  }
}
