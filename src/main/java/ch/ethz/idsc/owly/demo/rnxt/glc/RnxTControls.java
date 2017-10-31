// code by jph and jl
package ch.ethz.idsc.owly.demo.rnxt.glc;

import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.alg.Array;

// create radial controls
// class is intentionally public 
enum RnxTControls {
  ;
  // ---
  public static Flow stayPut(int u_length) {
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    return StateSpaceModels.createFlow(stateSpaceModel, Array.zeros(u_length));
  }
}
