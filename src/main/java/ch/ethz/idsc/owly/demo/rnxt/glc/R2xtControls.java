// code by jph and jl
package ch.ethz.idsc.owly.demo.rnxt.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.rn.R2Controls;
import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.alg.Array;

// create radial controls
// class is intentionally public 
public enum R2xtControls {
  ;
  // ---
  public static Collection<Flow> createRadial(final int num) {
    return R2Controls.createRadial(num);
    // StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    // List<Flow> list = new ArrayList<>();
    // for (Tensor angle : Range.of(0, num).multiply(DoubleScalar.of(2 * Math.PI / num))) {
    // Tensor u = Chop._10.of(Tensors.of(Cos.of(angle), Sin.of(angle), RealScalar.ONE));
    // list.add(StateSpaceModels.createFlow(stateSpaceModel, u));
    // }
    // return list;
  }

  public static Flow stayPut(int u_length) {
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    return StateSpaceModels.createFlow(stateSpaceModel, Array.zeros(u_length));
  }
}
