// code by jl and jph
package ch.ethz.idsc.owly.gui.ani;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.glc.adapter.IdentityWrap;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.Trajectories;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EmptyRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.EpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** omni-directional movement with constant speed */
public abstract class AbstractAnyEntity extends AbstractEntity {
  /** preserve 1[s] of the former trajectory */
  protected final Scalar delayHint;
  private final Scalar expandTime;
  protected final Parameters parameters;
  protected final Collection<Flow> controls;

  /** Constructor with default values for delayHint and expandTime
   * @param state
   * @param parameters
   * @param controls
   * @param episodeIntegrator */
  public AbstractAnyEntity(Tensor state, Parameters parameters, Collection<Flow> controls, EpisodeIntegrator episodeIntegrator) {
    this(state, parameters, controls, episodeIntegrator, RealScalar.ONE, RealScalar.of(0.5));
  }

  public AbstractAnyEntity(Tensor state, Parameters parameters, Collection<Flow> controls, EpisodeIntegrator episodeIntegrator, //
      Scalar delayHint, Scalar expandTime) {
    super(episodeIntegrator);
    this.parameters = parameters;
    this.controls = controls;
    this.delayHint = delayHint;
    this.expandTime = expandTime;
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.ANY;
  }

  @Override
  public Scalar delayHint() {
    return delayHint;
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    StateIntegrator stateIntegrator = createIntegrator();
    GoalInterface goalInterface = createGoal(goal);
    return new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, goalInterface);
  }

  /** @return the wanted Integrator for this Entity */
  protected abstract StateIntegrator createIntegrator();

  /** @param goal Goal locations in the StateSpace
   * @return the goalInterface for the right Entity */
  protected abstract GoalInterface createGoal(Tensor goal);

  /** Creates the GoalCheckHelperRegion
   * 
   * @param goal Tensor with center location of Goal
   * @return A Rwegion, which includes ALL GLcNodes, which could be followed by a trajectory, leading to the Goal */
  protected Region createGoalCheckHelp(Tensor goal) {
    return new InvertedRegion(EmptyRegion.INSTANCE);
  }

  /** Creates a new ObstacleQuery
   * 
   * @param region the Region of the environment
   * @param currentState the current state of the Entity
   * @return The new TRQ, which is the new Obstacle */
  protected TrajectoryRegionQuery updateObstacle(Region region, Tensor currentState) {
    return null;
  }

  Thread thread;
  public TrajectoryPlannerCallback trajectoryPlannerCallback;
  private List<TrajectorySample> head;
  GlcNode newRoot;
  Tensor goal;
  boolean switchGoalRequest = false;

  public void switchToGoal( //
      TrajectoryPlannerCallback trajectoryPlannerCallback, List<TrajectorySample> head, Tensor goal) {
    switchGoalRequest = true;
    this.goal = goal;
    try {
      while (switchGoalRequest) {
        Thread.sleep(500);
        System.out.println("block");
      }
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  public OptimalAnyTrajectoryPlanner trajectoryPlanner;

  // TODO SE2wrap remove to somewhere else
  public void startLife(Region environmentRegion, Tensor root) {
    TrajectoryRegionQuery trq = initializeObstacle(environmentRegion, root);
    trajectoryPlanner = (OptimalAnyTrajectoryPlanner) createTrajectoryPlanner(trq, root);
    trajectoryPlanner.represent = getWrap()::represent;
    trajectoryPlanner.switchRootToState(root); // setting start
    thread = new Thread(() -> {
      while (true) {
        long tic = System.nanoTime();
        head = (this.getFutureTrajectoryUntil(delayHint())); // Point on trajectory with delay from now
        // Rootswitch
        int index = getIndexOfLastNodeOf(head);
        Optional<GlcNode> optional = trajectoryPlanner.existsInTree(head.get(index).stateTime());
        if (!optional.isPresent())
          throw new RuntimeException();
        GlcNode newRoot = optional.get(); // getting last GlcNode in Head as root
        head = (head.subList(0, index + 1)); // cutting head to this Node
        int depthLimitIncrease = trajectoryPlanner.switchRootToNode(newRoot);
        parameters.increaseDepthLimit(depthLimitIncrease);
        // ObstacleUpdate
        TrajectoryRegionQuery newObstacle = updateObstacle(environmentRegion, head.get(0).stateTime().state());
        trajectoryPlanner.obstacleUpdate(newObstacle);
        if (switchGoalRequest) {
          // Goalswitch
          System.out.println("SwitchGoal Requested");
          GoalInterface goalInterface = createGoal(goal);
          Region goalCheckHelp = createGoalCheckHelp(goal);
          boolean result = trajectoryPlanner.changeToGoal(goalInterface, goalCheckHelp); // <- may take a while
          switchGoalRequest = false;
        } else {
          int iters = Expand.constTime(trajectoryPlanner, expandTime, parameters.getDepthLimit());
        }
        if (trajectoryPlannerCallback != null)
          trajectoryPlannerCallback.expandResult(head, trajectoryPlanner);
        try {
          Thread.sleep(10);
        } catch (Exception exception) {
          exception.printStackTrace();
        }
        System.err.println("Last iteration took: " + (System.nanoTime() - tic) * 1e-9 + "s");
      }
    });
    thread.start();
  }

  /** Wrap function, needs to be overwritten for StateSpaces with angles
   * @return */
  protected CoordinateWrap getWrap() {
    return IdentityWrap.INSTANCE;
  }

  /** creates the first Obstacle of the planner at initialization
   * 
   * @param region
   * @param currentState
   * @return ObstacleQuery */
  protected TrajectoryRegionQuery initializeObstacle(Region region, Tensor currentState) {
    return new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(region));
  }

  @Override
  protected List<TrajectorySample> resetAction(List<TrajectorySample> trajectory) {
    return trajectory;
  }

  private final int getIndexOfLastNodeOf(List<TrajectorySample> trajectory) {
    int index = trajectory.size() - 1;
    while (index >= 0) {
      Optional<GlcNode> optional = trajectoryPlanner.existsInTree(trajectory.get(index).stateTime());
      if (optional.isPresent())
        return index;
      index--; // going to previous statetime in traj
    }
    Trajectories.print(trajectory);
    throw new RuntimeException(); // no StateTime in trajectory corresponds to Node in Tree?
  }
}
