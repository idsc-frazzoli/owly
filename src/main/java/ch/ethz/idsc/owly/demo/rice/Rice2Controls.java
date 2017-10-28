// code by jph
package ch.ethz.idsc.owly.demo.rice;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Range;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.lie.AngleVector;

/** controls for position and velocity */
public enum Rice2Controls {
  ;
  /** @param mu coefficient, any real number
   * @param num amplitude resolution
   * @return */
  public static Collection<Flow> create1d(Scalar mu, int num) {
    StateSpaceModel stateSpaceModel = Rice2StateSpaceModel.of(mu);
    List<Flow> list = new ArrayList<>();
    for (Tensor u : Subdivide.of(DoubleScalar.of(-1), DoubleScalar.of(1), num))
      list.add(StateSpaceModels.createFlow(stateSpaceModel, Tensors.of(u)));
    return list;
  }

  /** radial
   * 
   * @param mu coefficient, any real number
   * @param seg amplitude resolution (0, 1]
   * @param num angular resolution
   * @return */
  public static Collection<Flow> create2d(Scalar mu, int seg, int num) {
    StateSpaceModel stateSpaceModel = Rice2StateSpaceModel.of(mu);
    Collection<Flow> collection = new HashSet<>();
    collection.add(StateSpaceModels.createFlow(stateSpaceModel, Array.zeros(2)));
    for (Tensor amp : Subdivide.of(0, 1, seg).extract(1, seg + 1))
      for (Tensor angle : Range.of(0, num).multiply(DoubleScalar.of(2 * Math.PI / num))) {
        Tensor u = AngleVector.of(angle.Get()).multiply(amp.Get());
        // System.out.println(u);
        collection.add(StateSpaceModels.createFlow(stateSpaceModel, u));
      }
    return collection;
  }
}
