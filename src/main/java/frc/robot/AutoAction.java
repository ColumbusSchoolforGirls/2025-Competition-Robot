package frc.robot;

public enum AutoAction {
   // LEAVE_ONLY,
   // INITIAL_DRIVE, //added for driving forward before turning towards the coral // TODO: change to drive 
   // TURN_TOWARD_REEF,
   // GO_TO_REEF,
   // SHOOT_CORAL,
   // STATION,
   // STOP,
   // GO_TO_REEF_AND_RAISE_ELEVATOR,
   // TURN,
   // DRIVE,
   // ADDITIONAL_DRIVE_ACTIONS,
   DRIVE,
   DRIVE_AND_ELEVATOR,
   TURN,
   ALIGN,
   SHOOT,
   STOP,
}

// coordinate system, 
// if start right place right 4 north
/*
 * drive (elevator up), turn, drive (align), shoot, drive (elevator down), turn, drive(backward) (align), drive (align)(elevator up)
 */