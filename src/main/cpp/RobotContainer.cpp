#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/commands/PPRamseteCommand.h>
#include "Commands/Autos.h"
#include "Commands/TestCommand.h"

using namespace pathplanner;

RobotContainer::RobotContainer() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
  PathPlannerTrajectory examplePath = PathPlanner::loadPath("Test Drive Forward", PathConstraints((units::meters_per_second_t)4,(units::meters_per_second_squared_t)3));
  // Reset odometry to the starting pose of the trajectory.
  m_drivetrain.ResetOdometry(examplePath.GetInitialPose());

  return std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {}));
  
  // no auto
  //return autos::TestAuto(&m_drivetrain);
}