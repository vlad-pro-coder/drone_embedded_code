#include "./All_drivers_Header.hpp"

using namespace std;

bool GPS::setupSerial(const char *port)
{
    serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0)
        return false;
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd, &tty) != 0)
        return false;
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag = 0;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    return tcsetattr(serial_fd, TCSANOW, &tty) == 0;
}
double GPS::convertToDecimal(const char *coord, const char dir)
{
    double val = atof(coord);
    int deg = (int)(val / 100);
    double min = val - deg * 100;
    double dec = deg + (min / 60.0);
    if (dir == 'S' || dir == 'W')
        dec *= -1;
    return dec;
}
GPS::GPS(const char *port = "/dev/serial0")
{
    if (!setupSerial(port))
    {
        cerr << "Eroare la deschiderea portului serial\n";
        serial_fd = -1;
    }
}
GPS::~GPS()
{
    if (serial_fd >= 0)
        close(serial_fd);
}
GPS_Data GPS::getCoordinates()
{
    if (serial_fd < 0)
        return false;
    char buffer[256];
    int index = 0;
    char c;
    while (true)
    {
        int n = read(serial_fd, &c, 1);
        if (n <= 0)
            return {-1,-1,-1};
        if (c == '\n')
        {
            buffer[index] = '\0';
            if (strncmp(buffer, "$GPRMC", 6) == 0)
            {
                char *fields[12];
                int fieldIndex = 0;
                fields[0] = strtok(buffer, ",");

                while (fields[fieldIndex] && fieldIndex < 11)
                {
                    fieldIndex++;
                    fields[fieldIndex] = strtok(NULL, ",");
                }
                if (fields[2] && fields[2][0] == 'A')
                {
                    if (fields[3] && fields[4] && fields[5] && fields[6])
                    {
                        double lat = convertToDecimal(fields[3], fields[4][0]);
                        double lon = convertToDecimal(fields[5], fields[6][0]);
                        return {1,lat,lon};
                    }
                }
            }
            index = 0;
        }
        else
        {
            if (index < 255)
                buffer[index++] = c;
        }
    }
}

/*int main() {
    GPS gps;
    double lat = 0.0, lon = 0.0;
    if (gps.getCoordinates(lat, lon)) {
        cout << "Coordonate GPS:\n";
        cout << "Latitudine: " << lat << "\n";
        cout << "Longitudine: " << lon << "\n";
    } else {
        cout << "Nu s-au putut obține coordonatele.\n";
    }

    return 0;
}*/