// code by jph
package ch.ethz.idsc.owly.demo.rnd;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Join;
import ch.ethz.idsc.tensor.alg.Range;
import ch.ethz.idsc.tensor.lie.AngleVector;
import ch.ethz.idsc.tensor.sca.Chop;

public enum R2dControls {
  ;
  public static Collection<Flow> createRadial(final int num) {
    GlobalAssert.that(2 < num); // otherwise does not cover plane
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    List<Flow> list = new ArrayList<>();
    for (Tensor a1 : Range.of(0, num).multiply(DoubleScalar.of(2 * Math.PI / num))) {
      for (Tensor a2 : Range.of(0, num).multiply(DoubleScalar.of(2 * Math.PI / num))) {
        Tensor u = Chop._10.of(Join.of(AngleVector.of(a1.Get()), AngleVector.of(a2.Get())));
        list.add(StateSpaceModels.createFlow(stateSpaceModel, u));
        // FIXME add zero movement (at least for 2nd agent)
      }
    }
    return list;
  }
}
