// code by jph
package ch.ethz.idsc.owly.gui.ani;

enum SimpleDemo {
  ;
  public static void main(String[] args) {
    OwlyAnimationFrame f = new OwlyAnimationFrame();
    f.add(new Rice2Animated());
    f.jFrame.setVisible(true);
  }
}
