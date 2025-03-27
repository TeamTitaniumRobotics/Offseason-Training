package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.Superstructure.SuperstructureState;

public class LED extends SubsystemBase {

  private static LEDPattern testRainbow =
      LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Inches.of(3));

  private static LEDPattern testRedBlack =
      LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Inches.of(30));

  public enum LEDStates {
    IDLE(LEDPattern.solid(Color.kGreen)),
    HOME(LEDPattern.solid(Color.kAliceBlue)),
    INTAKE_CORAL(LEDPattern.solid(Color.kAntiqueWhite)),
    READY_CORAL(LEDPattern.solid(Color.kBisque)),
    PRE_L1(LEDPattern.solid(Color.kAzure)),
    PRE_L2(LEDPattern.solid(Color.kBlanchedAlmond)),
    PRE_L3(),
    PRE_L4(),
    SCORE_CORAL(LED.testRainbow),
    SPIT_CORAL(),
    GROUND_INTAKE_STOW(),
    GROUND_INTAKE(),
    GROUND_INTAKE_HANDOFF(LEDPattern.solid(Color.kBlueViolet)),
    PRE_ALGAE_INTAKE_HIGH(),
    ALGAE_INTAKE_HIGH(),
    ALGAE_INTAKE_MIDDLE(LEDPattern.solid(Color.kSpringGreen)),
    ALGAE_INTAKE_STACK(),
    ALGAE_INTAKE_GROUND(),
    READY_ALGAE(),
    PRE_PROCESSOR(LEDPattern.solid(Color.kBrown)),
    PRE_NET(),
    SCORE_ALGAE(LED.testRainbow),
    SCORE_ALGAE_NET(LED.testRainbow),
    SPIT_ALGAE(),
    PRE_CLIMB(),
    CLIMB(LED.testRainbow),
    TEST_STATE();

    public final LEDPattern pattern; // The LEDPattern that the enum is supposed to store.

    
    LEDStates(LEDPattern pattern) {
      this.pattern = pattern;
    }

    LEDStates() {
      this.pattern = LEDPattern.solid(Color.kRed); // Default pattern of a solid red
    }
  }

//   public LEDStates isEquivalent(SuperstructureState superstructureState) {
//     for (LEDStates ledState : LEDStates.values()) {
//       if (ledState.name().equals(superstructureState.name())) {
//         return ledState; // Found a match
//       }
//     }
//     return null; // No match found
//   }

  private AddressableLED led;
  private AddressableLEDBuffer addressableLEDBuffer;
//   private Superstructure superstructure;

  public LED() {
    // this.superstructure = superstructure;
    led = new AddressableLED(9);
    // led.
    addressableLEDBuffer = new AddressableLEDBuffer(31); // 42
    led.setLength(31);
    led.start();

    // setDefaultCommand(
    //     runPattern(LED.testRainbow).withName("Rainbow Test")); // Test this rainbow color
  }

  private LEDStates LEDState;

  // private SuperstructureState lastState = null;

  @Override
  public void periodic() {
    // System.out.println("State:" + lastState);
    // System.out.println("Activates");
    // lastState = superstructure.getState();
    LEDState = LEDState.IDLE;

    System.out.println("State:" + LEDState);

    if (LEDState != null && LEDState.pattern != null) {
      LEDState.pattern.applyTo(addressableLEDBuffer);
      led.setData(addressableLEDBuffer);
    } else {
      System.out.println("ERROR: LEDState or its pattern is null!");
    }

    // runPattern(LEDState.getPattern());
    // led.setData(addressableLEDBuffer);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(addressableLEDBuffer)).ignoringDisable(true);
  }
}
