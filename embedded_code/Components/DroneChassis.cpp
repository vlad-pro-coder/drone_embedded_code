#include "./Components_Header.hpp"

DroneChassis::DroneChassis() : HeightCoefs(1.0, 0.0, 0.0), // example P/I/D values
                               YawCoefs(1.0, 0.0, 0.0),
                               PitchCoefs(1.0, 0.0, 0.0),
                               RollCoefs(1.0, 0.0, 0.0),
                               HeightPID(HeightCoefs),
                               YawPID(YawCoefs),
                               PitchPID(PitchCoefs),
                               RollPID(RollCoefs),
                               m1(1), m2(2), m3(3), m4(4)
{
}
void DroneChassis::drive(double outHeight, double outYaw, double outPitch, double outRoll)
{
    // Diminuator: normalize so all are within [-1, 1]
    double d = max(abs(outHeight) + abs(outYaw) + abs(outPitch) + abs(outRoll), 1);

    double m1power = (outHeight + outPitch - outRoll - outYaw) / d; // M1
    double m2power = (outHeight + outPitch + outRoll + outYaw) / d; // M2
    double m3power = (outHeight - outPitch - outRoll + outYaw) / d; // M3
    double m4power = (outHeight - outPitch + outRoll - outYaw) / d; // M4

    m1.setPower(m1power);
    m2.setPower(m2power);
    m3.setPower(m3power);
    m4.setPower(m4power);

    // TODO: send motorPowers[i] to ESCs
}

void DroneChassis::update()
{
    double currentYaw;
    double currentPitch;
    double currentRoll;
    double currentHeight;

    double cosYaw = cos(-currentYaw);
    double sinYaw = sin(-currentYaw);

    double bodyX = RollPID.getTargetPosition() * cosYaw - PitchPID.getTargetPosition() * sinYaw; // forward/backward → pitch target
    double bodyY = PitchPID.getTargetPosition() * sinYaw + RollPID.getTargetPosition() * cosYaw; // left/right → roll target

    double outHeight = HeightPID.calculatePower(currentHeight);
    double outYaw = HeightPID.calculatePower(currentYaw);
    double outPitch = HeightPID.calculatePower(currentPitch);
    double outRoll = HeightPID.calculatePower(currentYaw);
}

void DroneChassis::setYaw(double rads)
{
    YawPID.setTargetPosition(rads);
}
void DroneChassis::setRoll(double rads)
{
    RollPID.setTargetPosition(rads);
}
void DroneChassis::setPitch(double rads)
{
    PitchPID.setTargetPosition(rads);
}
void DroneChassis::setHeight(double DistMM)
{
    HeightPID.setTargetPosition(DistMM);
}