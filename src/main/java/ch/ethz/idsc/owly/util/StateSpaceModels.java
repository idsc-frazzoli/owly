// code by jph
package ch.ethz.idsc.owly.util;

import ch.ethz.idsc.tensor.Tensor;

public enum StateSpaceModels {
  ;
  // ---
  public static Flow createFlow(StateSpaceModel stateSpaceModel, Tensor u) {
    return new Flow() {
      @Override
      public Tensor at(Tensor x) {
        return stateSpaceModel.flow(x, u);
      }
    };
  }
}
