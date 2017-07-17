// code by jph
package ch.ethz.idsc.owly.gui;

import junit.framework.TestCase;

public class HueTest extends TestCase {
  @SuppressWarnings("unused")
  private void demoSimple() {
    for (int c = 0; c < 256; ++c) {
      double h = c / 256.;
      Hue hue = new Hue(h, 1, 1, 1);
      System.out.println(String.format("%d,%d,%d,%d", //
          hue.rgba.getRed(), //
          hue.rgba.getGreen(), //
          hue.rgba.getBlue(), //
          hue.rgba.getAlpha() //
      ));
    }
  }

  public void testDummy() {
    // ---
  }
}
