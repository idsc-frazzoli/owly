//code by jl
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.GoalInterface;

/** class contains static utility function that operate on instances of the {@link GoalInterface}
 * to test for the existence of Heuristics */
public enum HeuristicInspection {
  ;
  // TODO JAN: check with try and catch if correclty used
  /** Tests for the existence of a Heuristic
   * 
   * @param goalInterface the goalInterface to be tested
   * @return true if Heuristic exists, false if is does not exist */
  public static boolean hasHeuristic(GoalInterface goalInterface) {
    try {
      goalInterface.minCostToGoal(null); // If this throws no exception x was not used and the result is a constant. --> no heuristic
    } catch (Exception e) {
      return true; // Exception thrown x was used but error due to null -> heuristic
    }
    return false;
  }
}
