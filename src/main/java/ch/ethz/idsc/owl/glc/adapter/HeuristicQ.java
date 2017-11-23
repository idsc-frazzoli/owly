// code by jl
package ch.ethz.idsc.owl.glc.adapter;

import java.util.Objects;

import ch.ethz.idsc.owl.glc.core.HeuristicFunction;

/** class contains static utility function that operate on instances of the {@link HeuristicFunction}
 * to test for the implementation of a heuristic */
public enum HeuristicQ {
  ;
  /** tests for the implementation of a heuristic
   * 
   * @param heuristicFunction to inspect
   * @return true if a non-trivial heuristic is implemented, false if not */
  public static boolean of(HeuristicFunction heuristicFunction) {
    if (Objects.isNull(heuristicFunction))
      throw new NullPointerException();
    try {
      // if this throws no exception, x was not used and the result is a constant => no heuristic
      heuristicFunction.minCostToGoal(null);
    } catch (Exception exception) {
      // Exception thrown means x was used in function but error due to null => heuristic
      return true;
    }
    return false;
  }
}
