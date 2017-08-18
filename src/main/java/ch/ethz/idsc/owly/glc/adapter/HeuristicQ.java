//code by jl
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.state.CostFunction;

/** class contains static utility function that operate on instances of the {@link CostFunction}
 * to test for the existence of Heuristics */
public enum HeuristicQ {
  ;
  /** tests for the existence of a Heuristic
   * 
   * @param costFunction to inspect
   * @return true if Heuristic exists and is non-trivial, false if is does not exist */
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
