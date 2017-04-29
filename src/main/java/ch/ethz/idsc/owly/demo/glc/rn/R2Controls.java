// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.math.IdentityStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Range;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

class R2Controls {
  // ---
  public static Collection<Flow> createControls(final int num) {
    StateSpaceModel stateSpaceModel = new IdentityStateSpaceModel();
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Range.of(0, num).multiply(DoubleScalar.of(2 * Math.PI / num))) {
      Tensor u = Chop.of(Tensors.of(Cos.of(angle), Sin.of(angle)));
      list.add(StateSpaceModels.createFlow(stateSpaceModel, u));
    }
    return list;
  }
}
