// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

public enum Se2Controls {
  ;
  /** @param rate_max maximum turning rate in [rad/s]
   * @param num
   * @return num equidistant turning rates in the interval [-rate_max, +rate_max] */
  public static Collection<Flow> createControls(Scalar rate_max, int num) {
    if (num % 2 == 1)
      ++num;
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Subdivide.of(rate_max.negate(), rate_max, num)) {
      Tensor u = Tensors.of(angle);
      list.add(StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, u));
    }
    return Collections.unmodifiableList(list);
  }
}
