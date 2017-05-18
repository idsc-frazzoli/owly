// code by jph
package ch.ethz.idsc.owly.gui;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;

public enum Gui {
  ;
  public static OwlyFrame start() {
    OwlyFrame owlyFrame = new OwlyFrame();
    owlyFrame.jFrame.setVisible(true);
    return owlyFrame;
  }

  public static OwlyFrame glc(TrajectoryPlanner trajectoryPlanner) {
    OwlyFrame owlyFrame = new OwlyFrame();
    owlyFrame.setGlc(trajectoryPlanner);
    owlyFrame.jFrame.setVisible(true);
    return owlyFrame;
  }

  public static void rrts(RrtsNode rrtsNode, TransitionRegionQuery transitionRegionQuery) {
    OwlyFrame owlyFrame = new OwlyFrame();
    OwlyComponent c = owlyFrame.owlyComponent;
    c.renderElements = new RenderElements(rrtsNode, transitionRegionQuery);
    owlyFrame.jFrame.setVisible(true);
  }
}
