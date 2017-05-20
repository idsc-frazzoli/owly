// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Tensor;

/** utility functions related to {@link StateSpaceModel} */
public enum StateSpaceModels {
  ;
  // ---
  /** @param stateSpaceModel
   * @param u
   * @return flow defined by stateSpaceModel using control parameter u */
  public static Flow createFlow(StateSpaceModel stateSpaceModel, Tensor u) {
    return new Flow() {
      @Override
      public final Tensor at(Tensor x) {
        return stateSpaceModel.f(x, u);
      }

      @Override
      public final Tensor getU() {
        return u;
      }

      @Override
      public int hashCode() {
        return getU().hashCode();
      }

      @Override
      public boolean equals(Object object) {
        if (object instanceof Flow) {
          Flow flow = (Flow) object;
          return getU().equals(flow.getU());
        }
        return false;
      }
    };
  }
}
