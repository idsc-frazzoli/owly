// code by jph
package ch.ethz.idsc.owly.demo.glc.delta;

import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.tensor.RealScalar;

class DeltaDemo {
  public static void main(String[] args) throws Exception {
    TrajectoryPlanner trajectoryPlanner = DeltaHelper.createDefault(RealScalar.of(-.25)); // -.25 .5
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    while (trajectoryPlanner.getBest() == null && owlyFrame.jFrame.isVisible()) {
      Expand.maxSteps(trajectoryPlanner, 30);
      owlyFrame.setGlc(trajectoryPlanner);
      Thread.sleep(1);
    }
  }
}
