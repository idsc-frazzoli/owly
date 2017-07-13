// code by jph
package ch.ethz.idsc.owly.demo.twd;

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
import ch.ethz.idsc.tensor.sca.Floor;
import ch.ethz.idsc.tensor.sca.Sqrt;

/** two wheel drive */
public enum TwdControls {
  ;
  /** @param wheelspeeds_max the maximum absolut values for the rotationalspeed of each wheel.
   * @param num number of flows to produce
   * @return list of Flows */
  public static Collection<Flow> createControls1(TwdStateSpaceModel stateSpaceModel, int num) {
    int numSqr = Floor.of(Sqrt.of(RealScalar.of(num))).number().intValue();
    Tensor wheelspeeds_max = stateSpaceModel.getWheelspeeds_max();
    // TODO JONAS better way then floor
    List<Flow> list = new ArrayList<>();
    for (Tensor wl : Subdivide.of(wheelspeeds_max.Get(1).negate(), wheelspeeds_max.Get(0), numSqr)) {
      for (Tensor wr : Subdivide.of(wheelspeeds_max.Get(2).negate(), wheelspeeds_max.Get(1), numSqr)) {
        Tensor u = Tensors.of(wl, wr);
        list.add(StateSpaceModels.createFlow(stateSpaceModel, Tensors.of(wl, wr)));
      }
    }
    // |wl|<=1 implemented over argument.
    return list;
  }

  public static Collection<Flow> createControls2(TwdStateSpaceModel stateSpaceModel, Scalar wheelspeeds_max, int num) {
    @SuppressWarnings("unused")
    TwdStateSpaceModel test = new TwdStateSpaceModel(RealScalar.of(3), RealScalar.of(3), RealScalar.of(3));
    // TODO JONAS implement, zB |wl|+|wr|<=1
    throw new RuntimeException();
  }
}
