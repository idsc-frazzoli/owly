// code by jph
package ch.ethz.idsc.owly.demo.glc.delta;

import java.util.ArrayList;
import java.util.Collection;

import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Range;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

class DeltaControls {
  public static Collection<Flow> createControls(DeltaStateSpaceModel dssm, Scalar amp, int num) {
    Collection<Flow> collection = new ArrayList<>();
    for (Tensor angle : Range.of(0, num).multiply(DoubleScalar.of(2 * Math.PI / num))) {
      Tensor u = Chop.of(Tensors.of(Cos.of(angle), Sin.of(angle)).multiply(amp));
      collection.add(StateSpaceModels.createFlow(dssm, u));
    }
    return collection;
  }
}
