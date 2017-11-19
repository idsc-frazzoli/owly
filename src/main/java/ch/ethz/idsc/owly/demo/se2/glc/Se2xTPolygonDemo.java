// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.io.IOException;

import ch.ethz.idsc.owly.demo.rn.R2xTPolygonStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.ExamplePolygons;
import ch.ethz.idsc.owly.demo.util.SimpleTranslationFamily;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.se2.BijectionFamily;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Sin;

public class Se2xTPolygonDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    CarxTEntity carxTEntity = new CarxTEntity(new StateTime(Tensors.vector(6.75, 5.4, 1 + Math.PI), RealScalar.ZERO));
    owlyAnimationFrame.set(carxTEntity);
    // ---
    BijectionFamily shift = new SimpleTranslationFamily( //
        scalar -> Tensors.of(Sin.FUNCTION.apply(scalar.multiply(RealScalar.of(0.2))), RealScalar.ZERO));
    Region<StateTime> region = new R2xTPolygonStateTimeRegion( //
        ExamplePolygons.CORNER_TOP_LEFT, shift, () -> carxTEntity.getStateTimeNow().time());
    carxTEntity.obstacleQuery = new SimpleTrajectoryRegionQuery(region);
    owlyAnimationFrame.setObstacleQuery(carxTEntity.obstacleQuery);
    // owlyAnimationFrame.addRegionRender(imageRegion);
    owlyAnimationFrame.addBackground((RenderInterface) region);
    // ---
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) throws IOException {
    new Se2xTPolygonDemo().start();
  }
}
