// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.BorderLayout;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JToolBar;
import javax.swing.WindowConstants;

import ch.ethz.idsc.owly.gui.OwlyComponent;
import ch.ethz.idsc.owly.util.TimeKeeper;
import ch.ethz.idsc.tensor.Scalar;

public class OwlyAnimationFrame {
  public final JFrame jFrame = new JFrame();
  private final OwlyComponent owlyComponent = new OwlyComponent();
  private final JLabel jLabel = new JLabel();
  private final Timer timer = new Timer();
  // ---
  AnimationInterface ani; // TODO temporary

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
    TimerTask renderTask = new TimerTask() {
      @Override
      public void run() {
        Scalar now = timeKeeper.now();
        // System.out.println("rep "+);
        ani.integrate(now);
        owlyComponent.jComponent.repaint();
      }
    };
    timer.schedule(renderTask, 100, 100);
    jFrame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosing(WindowEvent windowEvent) {
        timer.cancel();
      }
    });
  }

  // public void configCoordinateOffset(int px, int py) {
  // owlyAnimationComponent.model2pixel.set(RealScalar.of(px), 0, 2);
  // owlyAnimationComponent.model2pixel.set(RealScalar.of(py), 1, 2);
  // }
  //
  public void add(AnimationInterface ani) {
    this.ani = ani;
    // owlyAnimationComponent.ani = ani;
  }
}
