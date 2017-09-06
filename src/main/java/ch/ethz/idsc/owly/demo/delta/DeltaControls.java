// code by jph
package ch.ethz.idsc.owly.demo.delta;

import java.util.ArrayList;
import java.util.Collection;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Range;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public enum DeltaControls {
  ;
  public static Collection<Flow> createControls(StateSpaceModel stateSpaceModel, Scalar amp, int num) {
    Collection<Flow> collection = new ArrayList<>();
    for (Tensor angle : Range.of(0, num).multiply(DoubleScalar.of(2 * Math.PI / num))) {
      Tensor u = Chop._10.of(Tensors.of(Cos.of(angle), Sin.of(angle)).multiply(amp));
      collection.add(StateSpaceModels.createFlow(stateSpaceModel, u));
    }
    return collection;
  }

  /** @param controls
   * @return */
  public static Scalar maxSpeed(Collection<Flow> controls) {
    return controls.stream() //
        .map(Flow::getU) //
        .map(Norm._2::ofVector) //
        .reduce(Max::of).get();
  }
}
