// code by jph
package ch.ethz.idsc.owly.demo.twd;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

/** two wheel drive */
public enum TwdControls {
  ;
  /** @param wheelspeeds_max the maximum absolut values for the rotationalspeed of each wheel.
   * @param num number of flows to produce
   * @return list of Flows */
  public static Collection<Flow> createControls1(TwdStateSpaceModel stateSpaceModel, int num) {
    // TODO JONAS better way then floor
    // int numSqr = Floor.of(Sqrt.of(RealScalar.of(num))).number().intValue();
    int numSqr = num; // TODO to be checked with theory: simulated inputs is no R²
    Scalar wheelspeed_max = stateSpaceModel.getWheelspeeds_max();
    List<Flow> list = new ArrayList<>();
    for (Tensor wl : Subdivide.of(wheelspeed_max.negate(), wheelspeed_max, numSqr)) {
      for (Tensor wr : Subdivide.of(wheelspeed_max.negate(), wheelspeed_max, numSqr)) {
        Tensor u = Tensors.of(wl, wr);
        list.add(StateSpaceModels.createFlow(stateSpaceModel, Tensors.of(wl, wr)));
      }
    }
    // |wl|<=wheelspeedmax, && |wr| <=wheelspeedmax implemented over argument.
    return list;
  }

  public static Collection<Flow> createControls2(TwdStateSpaceModel stateSpaceModel, int num) {
    @SuppressWarnings("unused")
    int numSqr = num; // TODO to be checked with theory: simulated inputs is no R²
    Scalar wheelspeed_max = stateSpaceModel.getWheelspeeds_max();
    List<Flow> list = new ArrayList<>();
    Tensor wlList = Subdivide.of(wheelspeed_max.negate(), wheelspeed_max, numSqr);
    Scalar stepSize = wlList.Get(1).subtract(wlList.Get(0));
    for (Tensor wl : wlList) {
      // |wl|+|wr|<=1
      Scalar wr = (wheelspeed_max.subtract(wl)).negate();
      while (Scalars.lessEquals(wr, wheelspeed_max.subtract(wl))) {
        Tensor u = Tensors.of((Scalar) wl, wr);
        list.add(StateSpaceModels.createFlow(stateSpaceModel, Tensors.of(wl, wr)));
        wr = wr.add(stepSize);
      }
    }
    return list;
  }
}
