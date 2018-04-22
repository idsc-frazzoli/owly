// code by jph
package ch.ethz.idsc.owly.demo.rice;

import junit.framework.TestCase;

public class Rice2dxTGearDemoTest extends TestCase {
  public void testSimple() throws Exception {
    Rice2dxTGearDemo rice2dxTGearDemo = new Rice2dxTGearDemo();
    rice2dxTGearDemo.start();
    Thread.sleep(100);
    rice2dxTGearDemo.owlyAnimationFrame.jFrame.setVisible(false);
  }
}
