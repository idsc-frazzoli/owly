// code by jph
package ch.ethz.idsc.owly.demo.glc.rice2;

import java.util.Collection;
import java.util.HashSet;

import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.alg.Range;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

class Rice2Controls {
  public static Collection<Flow> createControls(Scalar lambda, int seg, int num) {
    StateSpaceModel stateSpaceModel = new Rice2StateSpaceModel(lambda);
    Collection<Flow> collection = new HashSet<>();
    for (Tensor amp : Subdivide.of(ZeroScalar.get(), RealScalar.ONE, seg))
      for (Tensor angle : Range.of(0, num).multiply(DoubleScalar.of(2 * Math.PI / num))) {
        Tensor u = Chop.of(Tensors.of(Cos.of(angle), Sin.of(angle)).multiply(amp.Get()));
        collection.add(StateSpaceModels.createFlow(stateSpaceModel, u));
      }
    return collection;
  }
}
