// code by jl and jph
package ch.ethz.idsc.owl.gui.ani;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.data.Stopwatch;
import ch.ethz.idsc.owl.glc.adapter.GlcExpand;
import ch.ethz.idsc.owl.glc.adapter.IdentityWrap;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.Trajectories;
import ch.ethz.idsc.owl.glc.any.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.par.Parameters;
import ch.ethz.idsc.owl.math.CoordinateWrap;
import ch.ethz.idsc.owl.math.StateTimeTensorFunction;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.InvertedRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.Regions;
import ch.ethz.idsc.owl.math.state.EpisodeIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** omni-directional movement with constant speed */
public abstract class AbstractAnyEntity extends AbstractCircularEntity {
  /** preserve delayHint[s] of the former trajectory */
  protected final Scalar delayHint;
  private final Scalar expandTime;
  protected final Parameters parameters;
  protected final Collection<Flow> controls;

  /** Constructor with default values for delayHint and expandTime
   * @param state
   * @param parameters
   * @param controls
   * @param episodeIntegrator */
  public AbstractAnyEntity(StateTime state, Parameters parameters, //
      Collection<Flow> controls, EpisodeIntegrator episodeIntegrator) {
    this(parameters, controls, episodeIntegrator, RealScalar.ONE, RealScalar.of(0.5));
  }

  public AbstractAnyEntity( //
      // Tensor state, //
      Parameters parameters, //
      Collection<Flow> controls, //
      EpisodeIntegrator episodeIntegrator, //
      Scalar delayHint, Scalar expandTime) {
    super(episodeIntegrator);
    this.parameters = parameters;
    System.out.println("Partitions in unit length:" + parameters.getEta());
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
   * @return A Region, which includes ALL GLcNodes, which could be followed by a trajectory, leading to the Goal */
  protected Region<Tensor> createGoalCheckHelp(Tensor goal) {
    return new InvertedRegion(Regions.emptyRegion());
  }

  /** Creates a new ObstacleQuery
   * 
   * @param region the Region of the environment
   * @param currentState the current state of the Entity
   * @return The new TRQ, which is the new Obstacle */
  protected TrajectoryRegionQuery updateObstacle(Region<Tensor> region, StateTime currentState) {
    return null;
  }

  Thread thread;
  public TrajectoryPlannerCallback trajectoryPlannerCallback;
  private List<TrajectorySample> head;
  StateTime newRootStateTime;
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

  // TODO JONAS/JAN how to end thread, when OwlyAnimationFrame is closed
  public void startLife(Region<Tensor> environmentRegion, StateTime root) {
    TrajectoryRegionQuery trq = initializeObstacle(environmentRegion, root);
    // TrajectoryPlanner with Goal where he is currently
    trajectoryPlanner = (OptimalAnyTrajectoryPlanner) createTrajectoryPlanner(trq, root.state());
    trajectoryPlanner.represent = StateTimeTensorFunction.state(getWrap()::represent);
    trajectoryPlanner.switchRootToState(root); // setting start
    thread = new Thread(() -> {
      while (true) {
        Stopwatch stopwatch = Stopwatch.started();
        head = getFutureTrajectoryUntil(delayHint()); // Point on trajectory with delay from now
        // Rootswitch
        Trajectories.print(head);
        System.out.println("headsize " + head.size());
        int index = getIndexOfLastNodeOf(head);
        System.out.println("index    " + index);
        //
        Optional<GlcNode> optional = trajectoryPlanner.existsInTree(head.get(index).stateTime());
        newRootStateTime = head.get(0).stateTime();
        if (optional.isPresent())
          newRootStateTime = optional.get().stateTime(); // getting last GlcNode in Head as root
        System.out.println("NEW ROOT");
        System.out.println(newRootStateTime.toInfoString());
        head = head.subList(0, index + 1); // cutting head to this Node
        int depthLimitIncrease = trajectoryPlanner.switchRootToState(newRootStateTime);
        parameters.increaseDepthLimit(depthLimitIncrease);
        // ObstacleUpdate
        TrajectoryRegionQuery newObstacle = updateObstacle(environmentRegion, head.get(0).stateTime());
        trajectoryPlanner.obstacleUpdate(newObstacle);
        if (switchGoalRequest) {
          // Goalswitch
          System.out.println("SwitchGoal Requested");
          GoalInterface goalInterface = createGoal(goal);
          Region<Tensor> goalCheckHelp = createGoalCheckHelp(goal);
          // boolean result =
          trajectoryPlanner.changeToGoal(goalInterface, goalCheckHelp); // <- may take a while
          switchGoalRequest = false;
        } else {
          // int iters =
          GlcExpand.constTime(trajectoryPlanner, expandTime, parameters.getDepthLimit());
        }
        if (trajectoryPlannerCallback != null)
          trajectoryPlannerCallback.expandResult(head, trajectoryPlanner);
        try {
          Thread.sleep(10);
        } catch (Exception exception) {
          exception.printStackTrace();
        }
        stopwatch.stop();
        System.err.println("Last iteration took: " + stopwatch.display_seconds() + "s");
        stopwatch.resetToZero();
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
  protected TrajectoryRegionQuery initializeObstacle(Region<Tensor> oldEnvironmentRegion, StateTime currentStateTime) {
    return SimpleTrajectoryRegionQuery.timeInvariant(oldEnvironmentRegion);
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
    if (trajectory.size() == 1) { // no trajectory, drifting around
      return 0;
    }
    Trajectories.print(trajectory);
    throw new RuntimeException("no StateTime in trajectory corresponds to Node in Tree");
  }
}
