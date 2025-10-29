package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import frc.robot.subsystems.leds.LED.LEDState;

public class LedIOReal implements LedIO {
    private final CANdle leds = new CANdle(3, "rio");
    private LEDState currentState = LEDState.NOTHING;

    private final SolidColor solidColor = new SolidColor(8, 64);
    private final LarsonAnimation larsonAnimation = new LarsonAnimation(8, 64).withSlot(0);
    private final RainbowAnimation rainbowAnimation = new RainbowAnimation(8, 64).withSlot(0);
    private final SingleFadeAnimation singleFadeAnimation = new SingleFadeAnimation(8, 64).withSlot(0);

    public LedIOReal() {
        configureCANdle();
    }

    @Override
    public void requestState(LEDState nextState) {
        // define state priority ex. only show coral sensor status
        // if nothing else happening
        if (currentState == nextState) return;

        if ((nextState == LEDState.EMPTY || nextState == LEDState.HOLDING_CORAL)
                && (currentState != LEDState.EMPTY
                        && currentState != LEDState.HOLDING_CORAL
                        && currentState != LEDState.NOTHING)) return;

        leds.setControl(new EmptyAnimation(0));

        switch (nextState) {
            case EMPTY -> leds.setControl(solidColor.withColor(new RGBWColor(255, 0, 0, 0)));
            case HOLDING_CORAL -> leds.setControl(new SolidColor(8, 64).withColor(new RGBWColor(0, 255, 0)));
            case ALIGNING -> leds.setControl(rainbowAnimation.withBrightness(1.0).withDirection(AnimationDirectionValue.Forward).withFrameRate(0.1));
            case PLACING -> leds.setControl(singleFadeAnimation.withColor(new RGBWColor(0, 0, 255)).withFrameRate(0.2));
            case DISABLED -> leds.setControl(larsonAnimation.withColor(new RGBWColor(0, 0, 255)).withBounceMode(LarsonBounceValue.Center).withSize(8).withFrameRate(0.1));
            default -> {}
        }
        currentState = nextState;
    }

    private void configureCANdle() {
        CANdleConfiguration candleConfig = new CANdleConfiguration();
        candleConfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
        candleConfig.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
        candleConfig.LED.StripType = StripTypeValue.GRBW;
        candleConfig.LED.BrightnessScalar = 1.0;
    }
}
