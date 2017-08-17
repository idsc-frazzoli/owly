// code by jl and jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;
import java.util.List;
import java.util.ListIterator;
import java.util.Optional;

import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.Trajectories;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.OwlyLayer;
import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EmptyRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** omni-directional movement with constant speed */
public abstract class AbstractAnyEntity extends AbstractEntity {
  /** preserve 1[s] of the former trajectory */
  private static final Scalar DELAY_HINT = RealScalar.of(1);
  private static final Scalar EXPAND_TIME = RealScalar.of(0.5);
  protected final Parameters parameters;
  protected final Collection<Flow> controls;

  /** @param state initial position of entity */
  public AbstractAnyEntity(Tensor state, Parameters parameters, Collection<Flow> controls) {
    super(new SimpleEpisodeIntegrator( //
        SingleIntegratorStateSpaceModel.INSTANCE, //
        EulerIntegrator.INSTANCE, //
        new StateTime(state, RealScalar.ZERO)));
    this.parameters = parameters;
    this.controls = controls;
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.ANY;
  }

  @Override
  public Scalar delayHint() {
    return DELAY_HINT;
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
    GoalInterface goalInterface = createGoal(goal);
    return new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, goalInterface);
  }

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

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // indicate current position
      Tensor state = getStateTimeNow().state();
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(64, 128, 64, 192));
      graphics.fill(new Ellipse2D.Double(point.getX() - 2, point.getY() - 2, 7, 7));
    }
    { // indicate position 1[s] into the future
      Tensor state = getEstimatedLocationAt(DELAY_HINT);
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 128 - 64, 128 + 64));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
  }

  Thread thread;
  public TrajectoryPlannerCallback trajectoryPlannerCallback;
  List<TrajectorySample> head;
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

  public void startLife(Region environmentRegion, Tensor root) {
    TrajectoryRegionQuery trq = initializeObstacle(environmentRegion, root);
    trajectoryPlanner = (OptimalAnyTrajectoryPlanner) createTrajectoryPlanner(trq, root); // Tensors.vector(3, 4));
    trajectoryPlanner.switchRootToState(root); // setting start
    thread = new Thread(() -> {
      while (true) {
        long tic = System.nanoTime();
        head = this.getFutureTrajectoryUntil(delayHint()); // Point on trajectory with delay from now
        // Rootswitch
        int index = getIndexOfLastNodeOf(head);
        Optional<GlcNode> optional = trajectoryPlanner.existsInTree(head.get(index).stateTime());
        if (!optional.isPresent())
          throw new RuntimeException();
        GlcNode newRoot = optional.get(); // getting last GlcNode in Head as root
        head = head.subList(0, index + 1); // cutting head to this Node
        int depthLimitIncrease = trajectoryPlanner.switchRootToNode(newRoot);
        parameters.increaseDepthLimit(depthLimitIncrease);
        // ObstacleUpdate
        TrajectoryRegionQuery newObstacle = updateObstacle(environmentRegion, head.get(0).stateTime().state());
        trajectoryPlanner.ObstacleUpdate(newObstacle);
        if (switchGoalRequest) {
          // Goalswitch
          System.out.println("SwitchGoal Requested");
          GoalInterface goalInterface = createGoal(goal);
          Region goalCheckHelp = createGoalCheckHelp(goal);
          boolean result = trajectoryPlanner.changeToGoal(goalInterface, goalCheckHelp); // <- may take a while
          switchGoalRequest = false;
        } else {
          int iters = Expand.constTime(trajectoryPlanner, EXPAND_TIME, parameters.getDepthLimit());
        }
        if (trajectoryPlannerCallback != null)
          trajectoryPlannerCallback.expandResult(head, trajectoryPlanner);
        try {
          Thread.sleep(10);
        } catch (Exception exception) {
          exception.printStackTrace();
        }
        System.err.println("Last iteration took: " + (System.nanoTime()-tic)*1e-9 + "s");
      }
    });
    thread.start();
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
    ListIterator<TrajectorySample> iterator = trajectory.listIterator(trajectory.size()); // start at the back
    int index = trajectory.size() - 1;
    while (index >= 0) {
      Optional<GlcNode> optional = trajectoryPlanner.existsInTree(trajectory.get(index).stateTime());
      if (optional.isPresent())
        return index;
      index--; // going to previous statetime in traj
    }
    // TODO JONAS: change to next passed GlcNode?
    // TODO BUG JONAS: during movement no trajectory to new Goal
    Trajectories.print(trajectory);
    throw new RuntimeException(); // no StateTime in trajectory corresponds to Node in Tree?
  }
}
