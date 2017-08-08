// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.BorderLayout;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JToolBar;
import javax.swing.WindowConstants;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.data.TimeKeeper;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.Trajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.GoalRender;
import ch.ethz.idsc.owly.gui.ObstacleRender;
import ch.ethz.idsc.owly.gui.OwlyComponent;
import ch.ethz.idsc.owly.gui.RenderElements;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.TrajectoryRender;
import ch.ethz.idsc.owly.gui.misc.ImageRegionRender;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

// TODO class is in draft status, API not finalized 
public class OwlyAnimationFrame {
  public final JFrame jFrame = new JFrame();
  private final OwlyComponent owlyComponent = new OwlyComponent();
  private final JLabel jLabel = new JLabel();
  private final Timer timer = new Timer();
  // ---
  private final TrajectoryRender trajectoryRender = new TrajectoryRender(null);
  private final ObstacleRender obstacleRender = new ObstacleRender(null);
  private final GoalRender goalRender = new GoalRender(null);
  private final List<AnimationInterface> animationInterfaces = new LinkedList<>();
  /** reference to the entity that is controlled by the user */
  private AnimationInterface controllable = null;
  private TrajectoryRegionQuery obstacleQuery = EmptyTrajectoryRegionQuery.INSTANCE;

  public OwlyAnimationFrame() {
    { // install frame components
      JPanel jPanel = new JPanel(new BorderLayout());
      {
        JToolBar jToolBar = new JToolBar();
        jToolBar.setFloatable(false);
        {
          JButton jButton = new JButton("save2png");
          jButton.setToolTipText("file is created in Pictures/...");
          jToolBar.add(jButton);
        }
      }
      jPanel.add(owlyComponent.jComponent, BorderLayout.CENTER);
      jPanel.add(jLabel, BorderLayout.SOUTH);
      jFrame.setContentPane(jPanel);
    }
    jFrame.setBounds(100, 50, 800, 800); // default, can be changed if necessary
    jFrame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    owlyComponent.renderElements = new RenderElements();
    owlyComponent.renderElements.list.add(trajectoryRender);
    owlyComponent.renderElements.list.add(obstacleRender);
    owlyComponent.renderElements.list.add(goalRender);
    final TimerTask timerTask = new TimerTask() { // animation and repaint task
      final TimeKeeper timeKeeper = new TimeKeeper();

      @Override
      public void run() {
        Scalar now = timeKeeper.now();
        animationInterfaces.forEach(ani -> ani.integrate(now));
        owlyComponent.jComponent.repaint();
      }
    };
    timer.schedule(timerTask, 100, 50);
    jFrame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosing(WindowEvent windowEvent) {
        timer.cancel();
      }
    });
    // ---
    owlyComponent.jComponent.addMouseListener(new MouseAdapter() {
      MotionPlanWorker mpw = null;

      @Override
      public void mouseClicked(MouseEvent mouseEvent) {
        if (mouseEvent.getButton() == 1) {
          if (Objects.nonNull(mpw)) {
            mpw.flagShutdown();
            mpw = null;
          }
          if (controllable instanceof AbstractEntity) {
            AbstractEntity abstractEntity = (AbstractEntity) controllable;
            Tensor goal = owlyComponent.getMouseGoal();
            TrajectoryPlanner trajectoryPlanner = //
                abstractEntity.createTrajectoryPlanner(obstacleQuery, goal);
            mpw = new MotionPlanWorker(trajectoryPlannerCallback);
            mpw.start(abstractEntity.getFutureTrajectoryUntil(abstractEntity.delayHint()), trajectoryPlanner);
          }
        }
      }
    });
  }

  private final TrajectoryPlannerCallback trajectoryPlannerCallback = new TrajectoryPlannerCallback() {
    @Override
    public void expandResult(List<TrajectorySample> head, TrajectoryPlanner trajectoryPlanner) {
      Optional<GlcNode> optional = trajectoryPlanner.getBest();
      if (optional.isPresent()) {
        List<TrajectorySample> trajectory = new ArrayList<>();
        if (controllable instanceof AbstractEntity) {
          AbstractEntity abstractEntity = (AbstractEntity) controllable;
          List<TrajectorySample> tail = trajectoryPlanner.detailedTrajectoryTo(optional.get());
          trajectory = Trajectories.glue(head, tail);
          abstractEntity.setTrajectory(trajectory);
        }
        trajectoryRender.setTrajectory(trajectory);
      } else {
        System.err.println("NO TRAJECTORY BETWEEN ROOT TO GOAL");
      }
      StandardTrajectoryPlanner stp = (StandardTrajectoryPlanner) trajectoryPlanner;
      {
        TrajectoryRegionQuery trq = stp.getObstacleQuery();
        if (trq instanceof SimpleTrajectoryRegionQuery) {
          SimpleTrajectoryRegionQuery simpleTrajectoryRegionQuery = (SimpleTrajectoryRegionQuery) trq;
          Collection<StateTime> collection = simpleTrajectoryRegionQuery.getSparseDiscoveredMembers();
          obstacleRender.setCollection(new HashSet<>(collection));
        }
      }
      {
        TrajectoryRegionQuery trq = stp.getGoalQuery();
        if (trq instanceof SimpleTrajectoryRegionQuery) {
          SimpleTrajectoryRegionQuery simpleTrajectoryRegionQuery = (SimpleTrajectoryRegionQuery) trq;
          Collection<StateTime> collection = simpleTrajectoryRegionQuery.getSparseDiscoveredMembers();
          goalRender.setCollection(new HashSet<>(collection));
        }
      }
      owlyComponent.jComponent.repaint();
    }
  };

  public void set(AnimationInterface animationInterface) {
    GlobalAssert.that(animationInterfaces.isEmpty());
    add(animationInterface);
  }

  public void setObstacleQuery(TrajectoryRegionQuery obstacleQuery) {
    this.obstacleQuery = obstacleQuery;
  }

  public void addBackground(ImageRegion imageRegion) {
    addBackground(new ImageRegionRender(imageRegion));
  }

  public void addBackground(RenderInterface renderInterface) {
    owlyComponent.addDrawable(renderInterface);
  }

  private void add(AnimationInterface animationInterface) {
    if (Objects.isNull(controllable))
      controllable = animationInterface;
    // ---
    animationInterfaces.add(animationInterface);
    if (animationInterface instanceof RenderInterface) {
      RenderInterface renderInterface = (RenderInterface) animationInterface;
      owlyComponent.renderElements.list.add(renderInterface);
    }
  }

  public void configCoordinateOffset(int px, int py) {
    owlyComponent.setOffset(Tensors.vector(px, py));
  }
}
