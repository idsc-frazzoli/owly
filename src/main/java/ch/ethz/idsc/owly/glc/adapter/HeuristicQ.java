// code by jl
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.state.CostFunction;

/** class contains static utility function that operate on instances of the {@link CostFunction}
 * to test for the implementation of a heuristic */
public enum HeuristicQ {
  ;
  /** tests for the implementation of a heuristic
   * 
   * @param costFunction to inspect
   * @return true if a non-trivial heuristic is implemented, false if not */
  public static boolean of(CostFunction costFunction) {
    try {
      // if this throws no exception, x was not used and the result is a constant => no heuristic
      costFunction.minCostToGoal(null);
    } catch (Exception exception) {
      // Exception thrown means x was used in function but error due to null => heuristic
      return true;
    }
    return false;
  }
}
