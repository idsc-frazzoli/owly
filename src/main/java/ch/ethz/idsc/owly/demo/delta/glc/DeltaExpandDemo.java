// code by jph
package ch.ethz.idsc.owly.demo.delta.glc;

import ch.ethz.idsc.owly.demo.util.UserHome;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.io.GifSequenceWriter;

/** simple animation of small boat driving upstream, or downstream in a river delta
 * 
 * records to animated gif */
class DeltaExpandDemo {
  public static void main(String[] args) throws Exception {
    TrajectoryPlanner trajectoryPlanner = DeltaHelper.createDefault(RealScalar.of(.5)); // -.25 .5
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    GifSequenceWriter gsw = GifSequenceWriter.of(UserHome.Pictures("delta_s.gif"), 250);
    while (!trajectoryPlanner.getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
      Expand.maxSteps(trajectoryPlanner, 40);
      owlyFrame.setGlc(trajectoryPlanner);
      gsw.append(owlyFrame.offscreen());
      Thread.sleep(1);
    }
    int repeatLast = 6;
    while (0 < repeatLast--)
      gsw.append(owlyFrame.offscreen());
    gsw.close();
    System.out.println("created gif");
  }
}
