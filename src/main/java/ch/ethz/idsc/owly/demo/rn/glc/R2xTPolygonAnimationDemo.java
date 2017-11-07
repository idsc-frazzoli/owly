// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import ch.ethz.idsc.owly.demo.rn.R2xTPolygonStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.ExamplePolygons;
import ch.ethz.idsc.owly.demo.util.TranslationFamily;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.lie.AngleVector;
import ch.ethz.idsc.tensor.sca.Sin;

public class R2xTPolygonAnimationDemo implements DemoInterface {
  @SuppressWarnings("unused")
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    AbstractEntity abstractEntity = new R2xTEntity(Tensors.vector(0.2, 0.2));
    owlyAnimationFrame.set(abstractEntity);
    TranslationFamily shift = new TranslationFamily( //
        scalar -> Tensors.of(Sin.FUNCTION.apply(scalar.multiply(RealScalar.of(0.2))), RealScalar.ZERO));
    TranslationFamily circle = new TranslationFamily( //
        scalar -> AngleVector.of(scalar.multiply(RealScalar.of(0.2))));
    Region<StateTime> stateTimeRegion = new R2xTPolygonStateTimeRegion( //
        ExamplePolygons.CORNER_POINTY, circle, () -> abstractEntity.getStateTimeNow().time());
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery(stateTimeRegion));
    owlyAnimationFrame.addBackground((RenderInterface) stateTimeRegion);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new R2xTPolygonAnimationDemo().start();
  }
}
