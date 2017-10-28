// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.TimerTask;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.data.TimeKeeper;
import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.Trajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.TimerFrame;
import ch.ethz.idsc.owly.gui.ren.EtaRender;
import ch.ethz.idsc.owly.gui.ren.GoalRender;
import ch.ethz.idsc.owly.gui.ren.GridRender;
import ch.ethz.idsc.owly.gui.ren.ImageRegionRender;
import ch.ethz.idsc.owly.gui.ren.ObstacleRender;
import ch.ethz.idsc.owly.gui.ren.TrajectoryRender;
import ch.ethz.idsc.owly.gui.ren.TreeRender;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.rrts.adapter.SampledTransitionRegionQuery;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsPlanner;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

// EXPERIMENTAL API not finalized 
public class OwlyAnimationFrame extends TimerFrame {
  private final EtaRender etaRender = new EtaRender(Tensors.empty());
  private final TrajectoryRender trajectoryRender = new TrajectoryRender();
  private final ObstacleRender obstacleRender = new ObstacleRender(null);
  private final GoalRender goalRender = new GoalRender(null);
  private final TreeRender treeRender = new TreeRender(null);
  private final List<AnimationInterface> animationInterfaces = new LinkedList<>();
  /** reference to the entity that is controlled by the user */
  private AnimationInterface controllable = null;
  /** the obstacle query is set in {@link #setObstacleQuery(TrajectoryRegionQuery)}
   * it is intentionally set to null here lest the application forget */
  private TrajectoryRegionQuery obstacleQuery = null;

  public OwlyAnimationFrame() {
    geometricComponent.addRenderInterface(GridRender.INSTANCE);
    geometricComponent.addRenderInterface(etaRender);
    geometricComponent.addRenderInterface(trajectoryRender);
    geometricComponent.addRenderInterface(obstacleRender);
    geometricComponent.addRenderInterface(goalRender);
    geometricComponent.addRenderInterface(treeRender);
    { // periodic task for integration
      final TimerTask timerTask = new TimerTask() {
        TimeKeeper timeKeeper = new TimeKeeper();

        @Override
        public void run() {
          Scalar now = timeKeeper.now();
          animationInterfaces.forEach(ani -> ani.integrate(now));
        }
      };
      timer.schedule(timerTask, 100, 20);
    }
    // ---
    geometricComponent.jComponent.addMouseListener(new MouseAdapter() {
      MotionPlanWorker mpw = null;

      @Override
      public void mouseClicked(MouseEvent mouseEvent) {
        final int mods = mouseEvent.getModifiersEx();
        final int mask = MouseWheelEvent.CTRL_DOWN_MASK; // 128 = 2^7
        if (mouseEvent.getButton() == MouseEvent.BUTTON1) {
          if ((mods & mask) == 0) { // no ctrl pressed
            if (Objects.nonNull(mpw)) {
              mpw.flagShutdown();
              mpw = null;
            }
            if (controllable instanceof AbstractEntity) {
              AbstractEntity abstractEntity = (AbstractEntity) controllable;
              final Tensor goal = geometricComponent.getMouseSe2State();
              final List<TrajectorySample> head = //
                  abstractEntity.getFutureTrajectoryUntil(abstractEntity.delayHint());
              switch (abstractEntity.getPlannerType()) {
              case STANDARD: {
                TrajectoryPlanner trajectoryPlanner = //
                    abstractEntity.createTrajectoryPlanner(obstacleQuery, goal);
                mpw = new MotionPlanWorker(trajectoryPlannerCallback);
                mpw.start(head, trajectoryPlanner);
                break;
              }
              case ANY: {
                AbstractAnyEntity abstractAnyEntity = (AbstractAnyEntity) abstractEntity;
                abstractAnyEntity.switchToGoal(trajectoryPlannerCallback, head, goal);
                break;
              }
              case RRTS: {
                AbstractRrtsEntity abstractRrtsEntity = (AbstractRrtsEntity) abstractEntity;
                abstractRrtsEntity.startPlanner(trajectoryPlannerCallback, head, goal);
                break;
              }
              default:
                throw new RuntimeException();
              }
            }
          } else { // ctrl pressed
            System.out.println(geometricComponent.getMouseSe2State());
            if (controllable instanceof AbstractEntity) {
              @SuppressWarnings("unused")
              AbstractEntity abstractEntity = (AbstractEntity) controllable;
              // abstractEntity.resetStateTo(owlyComponent.getMouseGoal());
            }
          }
        }
      }
    });
  }

  public final TrajectoryPlannerCallback trajectoryPlannerCallback = new TrajectoryPlannerCallback() {
    @Override
    public void expandResult(List<TrajectorySample> head, TrajectoryPlanner trajectoryPlanner) {
      etaRender.setEta(trajectoryPlanner.getEta());
      Optional<GlcNode> optional = trajectoryPlanner.getFinalGoalNode();
      // test without heuristic
      if (optional.isPresent()) {
        List<TrajectorySample> trajectory = new ArrayList<>();
        if (controllable instanceof AbstractEntity) {
          AbstractEntity abstractEntity = (AbstractEntity) controllable;
          List<TrajectorySample> tail = trajectoryPlanner.detailedTrajectoryTo(optional.get());
          {
            Optional<GlcNode> temp = trajectoryPlanner.getBestOrElsePeek();
            List<StateTime> tempList = GlcNodes.getPathFromRootTo(temp.get());
            System.out.println("Root is: " + tempList.get(0).toInfoString());
          }
          // System.out.println("TAIL: <<<<<<<");
          // Trajectories.print(tail);
          trajectory = Trajectories.glue(head, tail);
          abstractEntity.setTrajectory(trajectory);
        }
        trajectoryRender.setTrajectory(trajectory);
      } else {
        System.err.println("NO TRAJECTORY BETWEEN ROOT TO GOAL");
      }
      {
        TrajectoryRegionQuery trq = trajectoryPlanner.getObstacleQuery();
        if (trq instanceof SimpleTrajectoryRegionQuery) {
          SimpleTrajectoryRegionQuery simpleTrajectoryRegionQuery = (SimpleTrajectoryRegionQuery) trq;
          Collection<StateTime> collection = simpleTrajectoryRegionQuery.getSparseDiscoveredMembers();
          obstacleRender.setCollection(new HashSet<>(collection));
        }
      }
      {
        TrajectoryRegionQuery trq = trajectoryPlanner.getGoalInterface();
        if (trq instanceof SimpleTrajectoryRegionQuery) {
          SimpleTrajectoryRegionQuery simpleTrajectoryRegionQuery = (SimpleTrajectoryRegionQuery) trq;
          Collection<StateTime> collection = simpleTrajectoryRegionQuery.getSparseDiscoveredMembers();
          goalRender.setCollection(new HashSet<>(collection));
        }
      }
      treeRender.setCollection(new ArrayList<>(trajectoryPlanner.getDomainMap().values()));
      // no repaint
    }

    @Override
    public void expandResult(List<TrajectorySample> head, RrtsPlanner rrtsPlanner, List<TrajectorySample> tail) {
      // TODO JAN code redundant to above... refactor
      List<TrajectorySample> trajectory = new ArrayList<>();
      if (controllable instanceof AbstractEntity) {
        AbstractEntity abstractEntity = (AbstractEntity) controllable;
        trajectory = Trajectories.glue(head, tail);
        abstractEntity.setTrajectory(trajectory);
      }
      trajectoryRender.setTrajectory(trajectory);
      {
        TransitionRegionQuery transitionRegionQuery = rrtsPlanner.getObstacleQuery();
        if (transitionRegionQuery instanceof SampledTransitionRegionQuery) {
          SampledTransitionRegionQuery strq = (SampledTransitionRegionQuery) transitionRegionQuery;
          obstacleRender.setCollection(new HashSet<>(strq.getDiscoveredMembers()));
        }
      }
      if (rrtsPlanner.getBest().isPresent()) {
        RrtsNode root = Nodes.rootFrom(rrtsPlanner.getBest().get());
        Collection<RrtsNode> collection = Nodes.ofSubtree(root);
        treeRender.setCollection(collection);
      }
    }
  };

  public void set(AnimationInterface animationInterface) {
    GlobalAssert.that(animationInterfaces.isEmpty());
    add(animationInterface);
  }

  /** modifies the obstacle region in between mouse-clicks
   * (so far only relevant for the standard planner)
   * 
   * @param obstacleQuery */
  public void setObstacleQuery(TrajectoryRegionQuery obstacleQuery) {
    this.obstacleQuery = obstacleQuery;
  }

  public void addBackground(ImageRegion imageRegion) {
    addBackground(new ImageRegionRender(imageRegion));
  }

  public void addBackground(RenderInterface renderInterface) {
    geometricComponent.addRenderInterfaceBackground(renderInterface);
  }

  private void add(AnimationInterface animationInterface) {
    if (Objects.isNull(controllable))
      controllable = animationInterface;
    // ---
    animationInterfaces.add(animationInterface);
    if (animationInterface instanceof RenderInterface) {
      RenderInterface renderInterface = (RenderInterface) animationInterface;
      geometricComponent.addRenderInterface(renderInterface);
    }
  }
}
