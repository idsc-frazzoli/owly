// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.BorderLayout;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JToolBar;
import javax.swing.WindowConstants;

import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ObstacleRender;
import ch.ethz.idsc.owly.gui.OwlyComponent;
import ch.ethz.idsc.owly.gui.RenderElements;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.TrajectoryRender;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.util.TimeKeeper;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class OwlyAnimationFrame {
  public final JFrame jFrame = new JFrame();
  private final OwlyComponent owlyComponent = new OwlyComponent();
  private final JLabel jLabel = new JLabel();
  private final Timer timer = new Timer();
  // ---
  TrajectoryRender trajectoryRender = new TrajectoryRender(null);
  // Collection<StateTime> obstacles = new HashSet<>();
  ObstacleRender obstacleRender = new ObstacleRender(null);
  List<AnimationInterface> animationInterfaces = new LinkedList<>(); // TODO temporary

  public OwlyAnimationFrame() {
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
    jFrame.setBounds(100, 50, 800, 800);
    jFrame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    TimeKeeper timeKeeper = new TimeKeeper();
    owlyComponent.renderElements = new RenderElements();
    owlyComponent.renderElements.list.add(trajectoryRender);
    owlyComponent.renderElements.list.add(obstacleRender);
    TimerTask timerTask = new TimerTask() {
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
    Region region = new R2NoiseRegion(.3);
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(region));
    owlyComponent.jComponent.addMouseListener(new MouseAdapter() {
      @Override
      public void mouseClicked(MouseEvent mouseEvent) {
        if (mouseEvent.getButton() == 1) {
          Tensor goal = owlyComponent.toModel(mouseEvent.getPoint());
          MotionPlanWorker mpw = new MotionPlanWorker(trajectoryPlannerCallback);
          Rice2Entity rice2Entity = (Rice2Entity) animationInterfaces.get(0);
          mpw.start(rice2Entity.episodeIntegrator.tail().x(), goal, obstacleQuery);
        }
      }
    });
  }

  TrajectoryPlannerCallback trajectoryPlannerCallback = new TrajectoryPlannerCallback() {
    @Override
    public void hasTrajectoryPlanner(TrajectoryPlanner trajectoryPlanner) {
      System.out.println("finished");
      // long toc = System.nanoTime();
      // System.out.println(iters + " " + ((toc - tic) * 1e-9));
      Optional<GlcNode> optional = trajectoryPlanner.getBest();
      if (optional.isPresent()) {
        List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
        Trajectories.print(trajectory);
      }
      trajectoryRender.setTrajectoryPlanner(trajectoryPlanner);
      StandardTrajectoryPlanner stp = (StandardTrajectoryPlanner) trajectoryPlanner;
      TrajectoryRegionQuery trq = stp.getObstacleQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery simpleTrajectoryRegionQuery = (SimpleTrajectoryRegionQuery) trq;
        Collection<StateTime> collection = simpleTrajectoryRegionQuery.getSparseDiscoveredMembers();
        obstacleRender.setCollection(new HashSet<>(collection));
      }
      // trajectoryRender
      owlyComponent.jComponent.repaint();
    }
  };

  public void add(AnimationInterface animationInterface) {
    animationInterfaces.add(animationInterface);
    if (animationInterface instanceof RenderInterface)
      owlyComponent.renderElements.list.add((RenderInterface) animationInterface);
  }
}
