// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import ch.ethz.idsc.owly.demo.rn.R2xTPolygonStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class R2xTPolygonAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    owlyAnimationFrame.set(new R2xTEntity(Tensors.vector(0.2, 0.2)));
    Tensor polygon = Tensors.matrix(new Number[][] { //
        { 3, 0 }, //
        { 4, 0 }, //
        { 4, 6 }, //
        { 1, 6 }, //
        { 1, 3 }, //
        { 3, 3 } //
    });
    StateTimeRegion stateTimeRegion = new R2xTPolygonStateTimeRegion(polygon);
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery(stateTimeRegion));
    owlyAnimationFrame.addBackground((RenderInterface) stateTimeRegion);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new R2xTPolygonAnimationDemo().start();
  }
}
