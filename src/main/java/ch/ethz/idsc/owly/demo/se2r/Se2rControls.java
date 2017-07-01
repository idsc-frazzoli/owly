// code by jph
package ch.ethz.idsc.owly.demo.se2r;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

public enum Se2rControls {
  ;
  /** @param angle of steering
   * @param speed, positive for forward, and negative for backward
   * @return */
  public static Flow create(Scalar angle, Scalar speed) {
    return StateSpaceModels.createFlow(Se2rStateSpaceModel.INSTANCE, Tensors.of(angle, speed));
  }

  public static Collection<Flow> createControls(Scalar angle_max, int num) {
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Subdivide.of(angle_max.negate(), angle_max, num)) {
      list.add(create(angle.Get(), RealScalar.ONE));
      list.add(create(angle.Get(), RealScalar.ONE.negate()));
    }
    return list;
  }
}
