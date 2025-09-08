#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <vector>
#include <cstring>
#include <cmath>
#include <tuple>

#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_MODE   SPI_MODE_0
#define SPI_SPEED  3000000   // 3 MHz
#define SPI_BITS   8

#define GPIO_CHIP "/dev/gpiochip0"
#define INT_LINE  25   // BCM GPIO25
#define PS0 17
#define PS1 27
#define RST 24

using namespace std;

float s16_to_float(int16_t val) { return val / 16384.0f; }

void send_rotation_vector_enable(int spi_fd) {
    uint8_t RequestRotationVectorReport[] = { 
        0x15, 0x00, 0x02, 0x00,   0xFD, 0x28,   0x00, 0x00, 0x00,   0x20, 0x4e,    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    struct spi_ioc_transfer tr{};
    tr.tx_buf = (unsigned long)RequestRotationVectorReport;
    tr.rx_buf = 0;
    tr.len = sizeof(RequestRotationVectorReport);
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = SPI_BITS;

    if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0)
        perror("SPI_IOC_MESSAGE send feature");
    else
        cout << "Rotation vector feature report enabled\n";
}

tuple<double,double,double> quatToEuler(double w,double x,double y,double z) {
    double ysqr=y*y;
    double t0=2*(w*x+y*z), t1=1-2*(x*x+ysqr), roll=atan2(t0,t1);
    double t2=2*(w*y-z*x); t2=t2>1?1:t2; t2=t2<-1?-1:t2; double pitch=asin(t2);
    double t3=2*(w*z+x*y), t4=1-2*(ysqr+z*z), yaw=atan2(t3,t4);
    return make_tuple(yaw,pitch,roll);
}

int main() {
    gpiod_chip* chip = gpiod_chip_open(GPIO_CHIP);
    if(!chip){ perror("gpiod_chip_open"); return 1; }

    gpiod_line* ps0 = gpiod_chip_get_line(chip, PS0);
    gpiod_line* ps1 = gpiod_chip_get_line(chip, PS1);
    gpiod_line* rst = gpiod_chip_get_line(chip, RST);
    gpiod_line* int_line = gpiod_chip_get_line(chip, INT_LINE);

    gpiod_line_request_output(ps0,"bno085",0);
    gpiod_line_request_output(ps1,"bno085",0);
    gpiod_line_request_output(rst,"bno085",0);
    gpiod_line_request_input(int_line,"bno085-int");

    gpiod_line_set_value(ps0,1);
    gpiod_line_set_value(ps1,1);
    usleep(100000);
    gpiod_line_set_value(rst,1);
    cout<<"Sensor should now be in SPI mode.\n";

    int spi_fd=open(SPI_DEVICE,O_RDWR);
    if(spi_fd<0){ perror("Failed to open SPI device"); return 1; }

    uint8_t mode=SPI_MODE, bits=SPI_BITS; uint32_t speed=SPI_SPEED;
    ioctl(spi_fd,SPI_IOC_WR_MODE,&mode);
    ioctl(spi_fd,SPI_IOC_WR_BITS_PER_WORD,&bits);
    ioctl(spi_fd,SPI_IOC_WR_MAX_SPEED_HZ,&speed);

    usleep(50000);
    send_rotation_vector_enable(spi_fd);

    cout<<"Waiting for data...\n";

    while(true){
        int val=gpiod_line_get_value(int_line);
        if(val<0){ perror("gpiod get value"); break; }
        if(val==1){ usleep(50000); continue; }

        // Read header
        uint8_t tx[4]={0,0,0,0}, rx[4]={0};
        struct spi_ioc_transfer tr{};
        tr.tx_buf=(unsigned long)tx;
        tr.rx_buf=(unsigned long)rx;
        tr.len=4;
        tr.speed_hz=SPI_SPEED;
        tr.bits_per_word=SPI_BITS;

        if(ioctl(spi_fd,SPI_IOC_MESSAGE(1),&tr)<0){ perror("SPI header"); break; }

        int packet_len=((rx[1]<<8) | rx[0]) & 0x7FFF;
        cout<<packet_len<<'\n';
        if(packet_len<=4 || packet_len>512) continue;

        int payload_len=packet_len-4;
        vector<uint8_t> buffer(payload_len);
        memset(tx,0,4);
        tr.tx_buf=(unsigned long)tx;
        tr.rx_buf=(unsigned long)buffer[0];
        tr.len=payload_len;
        if(ioctl(spi_fd,SPI_IOC_MESSAGE(1),&tr)<0){ perror("SPI payload"); break; }

        if(payload_len>=14){
            int16_t i16[4];
            for(int i=0;i<4;i++)
                i16[i]=buffer[5+i*2] | (buffer[6+i*2]<<8);

            float qw=s16_to_float(i16[0]), qx=s16_to_float(i16[1]),
                  qy=s16_to_float(i16[2]), qz=s16_to_float(i16[3]);

            cout<<fixed<<setprecision(4);
            cout<<"Quaternion: ["<<qw<<","<<qx<<","<<qy<<","<<qz<<"]\n";

            auto [yaw,pitch,roll]=quatToEuler(qw,qx,qy,qz);
            cout<<"Yaw: "<<yaw*180/M_PI<<" Pitch: "<<pitch*180/M_PI<<" Roll: "<<roll*180/M_PI<<"\n";
        }
    }

    gpiod_chip_close(chip);
    close(spi_fd);
    return 0;
}
