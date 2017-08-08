// code by jph & jl
package ch.ethz.idsc.owly.demo.deltaxt;

import java.util.ArrayList;
import java.util.Collection;

import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Range;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public enum DeltaxtControls {
  ;
  public static Collection<Flow> createControls(DeltaxtStateSpaceModel dssm, Scalar amp, int num) {
    Collection<Flow> collection = new ArrayList<>();
    for (Tensor angle : Range.of(0, num).multiply(DoubleScalar.of(2 * Math.PI / num))) {
      Tensor u = Chop._10.of(Tensors.of(Cos.of(angle).multiply(amp), Sin.of(angle).multiply(amp), RealScalar.ZERO));
      collection.add(StateSpaceModels.createFlow(dssm, u));
    }
    return collection;
  }
}
