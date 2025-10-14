#include "./Components_Header.hpp"

DroneChassis::DroneChassis() : HeightCoefs(0.0, 0.0, 0.0), // example P/I/D values
                               YawCoefs(0.2, 0.0, 0.0),
                               PitchCoefs(0.4, 0.0, 0.0),
                               RollCoefs(0.4, 0.0, 0.0),
                               HeightPID(HeightCoefs),
                               YawPID(YawCoefs),
                               PitchPID(PitchCoefs),
                               RollPID(RollCoefs),
                               fr(23), fl(25), br(24), bl(22)
{

    YawPID.setTargetPosition(0);
    RollPID.setTargetPosition(0);
    PitchPID.setTargetPosition(0);
    HeightPID.setTargetPosition(0);

    fr.setPower(0);
    fl.setPower(0);
    bl.setPower(0);
    br.setPower(0);

}

void DroneChassis::drive(double outHeight, double outYaw, double outPitch, double outRoll)
{
    // Diminuator: normalize so all are within [0, 1]
    double d = max(abs(outHeight) + abs(outYaw) + abs(outPitch) + abs(outRoll), 1.0);

    double frpower = (outHeight + outPitch - outRoll - outYaw) / d;
    double flpower = (outHeight + outPitch + outRoll + outYaw) / d;
    double brpower = (outHeight - outPitch - outRoll + outYaw) / d;
    double blpower = (outHeight - outPitch + outRoll - outYaw) / d;

    fr.setPower(frpower);
    fl.setPower(flpower);
    br.setPower(brpower);
    bl.setPower(blpower);

    cout << "\033[s\033[2;1H\033[K";

    cout << "Motor FL: " << flpower
         << "  FR: " << frpower
         << "  BL: " << blpower
         << "  BR: " << brpower;

    // Restore cursor and flush
    cout << "\033[u" << flush;

    // TODO: send motorPowers[i] to ESCs
}

void DroneChassis::update()
{
    tuple<double,double,double> results = imu.getAngles();
    double currentYaw   = get<0>(results);
    double currentPitch = get<1>(results);
    double currentRoll  = get<2>(results);

    double currentHeight = 0;//sensor.attr("get_distance")();

    double pitchError = getAngleDifference(this->TargetPitch, toRadians(currentPitch));
    double rollError  = getAngleDifference(this->TargetRoll, toRadians(currentRoll));
    double yawError   = getAngleDifference(this->TargetYaw, toRadians(currentYaw));
    double heightError = this->TargetHeight - currentHeight;

    // If world-relative: rotate x/y (forward/left) to body frame
    /*double normalizedYaw = currentYaw;
    if(normalizedYaw < 0) normalizedYaw += M_PI * 2;
    if(normalizedYaw > 2*M_PI) normalizedYaw -= M_PI * 2;*/

    double cosYaw = cos(toRadians(currentYaw));
    double sinYaw = sin(toRadians(currentYaw));

    double bodyX =  cosYaw * pitchError + sinYaw * rollError;
    double bodyY =  sinYaw * pitchError - cosYaw * rollError;

    // Feed PID with errors, target = 0
    double outPitch = PitchPID.calculatePower(bodyX);
    double outRoll  = RollPID.calculatePower(bodyY);
    double outYaw   = YawPID.calculatePower(yawError);
    double outHeight = HeightPID.calculatePower(heightError);

    cout << "\033[s\033[1;1H\033[K";
            cout << "Yaw: " << currentYaw
                 << " Pitch: " << currentPitch
                 << " Roll: " << currentRoll
                 << " | Diff Yaw: " << yawError
                 << " Diff Pitch: " << bodyX
                 << " Diff Roll: " << bodyY;
            cout << "\033[u" << flush;

    drive(outHeight, outYaw, outPitch, outRoll);
}

void DroneChassis::setYaw(double rads)
{
    this->TargetYaw = rads;
}
void DroneChassis::setRoll(double rads)
{
    this->TargetRoll = rads;
}
void DroneChassis::setPitch(double rads)
{
    this->TargetPitch = rads;
}
void DroneChassis::setHeight(double DistMM)
{
    this->TargetHeight = DistMM;
}

double DroneChassis::toRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}

double DroneChassis::getAngleDifference(double target, double current) {
    double diff = normalizeToMinusPiToPi(target - current);
    return diff;
}

double DroneChassis::normalizeToMinusPiToPi(double angle) {
    const double TWO_PI = 2.0 * M_PI;
    angle = fmod(angle + M_PI, TWO_PI);
    if (angle < 0)
        angle += TWO_PI;
    return angle - M_PI;
}