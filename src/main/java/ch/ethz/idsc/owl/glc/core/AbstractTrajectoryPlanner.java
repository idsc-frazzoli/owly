// code by bapaden, jph, and jl
package ch.ethz.idsc.owl.glc.core;

import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.owl.data.Stopwatch;
import ch.ethz.idsc.owl.glc.any.AbstractAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.std.PlannerConstraint;
import ch.ethz.idsc.owl.glc.std.StandardTrajectoryPlanner;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.tensor.Tensor;

/** planner is shared between
 * {@link StandardTrajectoryPlanner} and {@link AbstractAnyTrajectoryPlanner} */
public abstract class AbstractTrajectoryPlanner extends TrajectoryPlanner {
  protected final StateIntegrator stateIntegrator;
  private /* not final */ PlannerConstraint plannerConstraint;
  private /* not final */ GoalInterface goalInterface;
  protected transient Stopwatch integratorWatch = Stopwatch.stopped();
  protected transient Stopwatch processCWatch = Stopwatch.stopped();

  protected AbstractTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      PlannerConstraint plannerConstraint, //
      GoalInterface goalInterface) {
    super(eta);
    GlobalAssert.that(Objects.nonNull(plannerConstraint));
    this.stateIntegrator = stateIntegrator;
    this.plannerConstraint = plannerConstraint;
    this.goalInterface = goalInterface;
  }

  @Override
  public final StateIntegrator getStateIntegrator() {
    return stateIntegrator;
  }

  @Override
  public final PlannerConstraint getPlannerConstraint() {
    return plannerConstraint;
  }

  public final void setPlannerConstraint(PlannerConstraint plannerConstraint) {
    this.plannerConstraint = plannerConstraint;
  }

  @Override
  public final GoalInterface getGoalInterface() {
    return goalInterface;
  }

  protected final void setGoalInterface(GoalInterface goalInterface) {
    this.goalInterface = goalInterface;
  }

  protected final boolean isInsideGoal(List<StateTime> trajectory) {
    return goalInterface.firstMember(trajectory).isPresent();
  }

  public void printTimes() {
    System.out.println("Times for " + getClass().getSimpleName());
    System.out.println("Integrator took: " + integratorWatch.display_seconds());
    System.out.println("processing C took: " + processCWatch.display_seconds());
    integratorWatch = Stopwatch.stopped();
    processCWatch = Stopwatch.stopped();
  }
}
