// code by ?
package ch.ethz.idsc.owly.demo.glc.delta;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;

class DeltaGlcDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RationalScalar.of(10, 1);
    TrajectoryPlanner trajectoryPlanner = DeltaHelper.createGlc(RealScalar.of(-0.25), resolution);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    // TODO build depthlimit in for loops
    while (!trajectoryPlanner.getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
      Expand.maxSteps(trajectoryPlanner, 30);
      owlyFrame.setGlc(trajectoryPlanner);
      List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
      Thread.sleep(1);
      if (trajectoryPlanner.getQueue().isEmpty()) {
        break;
      }
    }
  }
}
