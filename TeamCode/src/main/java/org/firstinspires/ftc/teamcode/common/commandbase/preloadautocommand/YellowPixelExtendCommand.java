package org.firstinspires.ftc.teamcode.common.commandbase.preloadautocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;

public class YellowPixelExtendCommand extends SequentialCommandGroup {
    public YellowPixelExtendCommand() {
        super(
                new ArmCommand(0.2),
                new PivotCommand(0.29),
                new WaitCommand(350),
                new ExtensionCommand(450),
                new WaitCommand(1000)
        );
    }
}