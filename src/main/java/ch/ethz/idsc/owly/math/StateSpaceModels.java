// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Tensor;

/** utility functions related to {@link StateSpaceModel} */
// TODO JAN since this class depends on Flow, the current location is not ideal
// ... consider moving to ch.ethz.idsc.owly.math.state ?
public enum StateSpaceModels {
  ;
  // ---
  /** creates an implementation of flow.
   * flow is made identifiable via u.
   * 
   * @param stateSpaceModel
   * @param u
   * @return flow defined by stateSpaceModel using control parameter u */
  public static Flow createFlow(StateSpaceModel stateSpaceModel, Tensor u) {
    Tensor u_unmodifiable = u.unmodifiable();
    return new Flow() {
      @Override
      public final Tensor at(Tensor x) {
        return stateSpaceModel.f(x, u);
      }

      @Override
      public final Tensor getU() {
        return u_unmodifiable;
      }
      // ---
      /** FLOW IMPLEMENTATION INTENTIONALLY DOES NOT OVERRIDE hashCode(), and equals(...)
       * 
       * FLOWS SHOULD BE DIFFERENTIATED OUTSIDE CLASS VIA THE STATE SPACE MODEL AND THE CONTROL U's */
    };
  }
}
