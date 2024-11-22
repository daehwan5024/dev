#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#define L_O 33 // motor1 offset
#define L_H 57.5 // motor2 offset
#define L_T 150 // thigh length
#define L_C 150 // calf length


// returns motor offset based on x, y, z value of end feet
void invDy(double x_end_L, double y_end_L, double z_end_L, double x_end_R, double y_end_R, double z_end_R, double *angles){
    double th3_L = acos((x_end_L*x_end_L+(y_end_L-L_O)*(y_end_L-L_O)+z_end_L*z_end_L-L_H*L_H-L_T*L_T-L_C*L_C)/(2*L_T*L_C));
    double AL = L_T+L_C*cos(th3_L);
    double BL = L_C*sin(th3_L);
    double th2_L = asin(-x_end_L/(sqrt(AL*AL+BL*BL))) - atan(BL/AL);
    double CL = L_H;
    double DL = L_T*cos(th2_L)+L_C*cos(th2_L+th3_L);
    double th1_L = asin(-z_end_L/sqrt(CL*CL+DL*DL))-atan(DL/CL);
    
    double th3_R = acos((x_end_R*x_end_R+(y_end_R+L_O)*(y_end_L+L_O)+z_end_R*z_end_R-L_H*L_H-L_T*L_T-L_C*L_C)/(2*L_T*L_C));
    double AR = L_T+L_C*cos(th3_R);
    double BR = L_C*sin(th3_R);
    double th2_R = asin(-x_end_R/(sqrt(AR*AR+BR*BR))) - atan(BR/AR);
    double CR = L_H;
    double DR = L_T*cos(th2_R)+L_C*cos(th2_R+th3_R);
    double th1_R = asin(-z_end_R/sqrt(CR*CR+DR*DR))-atan(DR/CR);

    angles[0] = th1_L;
    angles[1] = th2_L;
    angles[2] = th3_L;
    angles[3] = th1_R;
    angles[4] = th2_R;
    angles[5] = th3_R;
}

int main() {
    int uart_fd;
    char uart_device[] = "/dev/serial0"; // UART 장치 경로(Raspberry Pi 기본 UART)
    int baud_rate = 9600; // STM32와 일치해야 함
    int number_to_send = 12345; // 보낼 숫자 예시

    // UART 초기화
    if ((uart_fd = serialOpen(uart_device, baud_rate)) < 0) {
        fprintf(stderr, "Unable to open UART device: %s\n", uart_device);
        return 1;
    }

    // WiringPi 초기화
    if (wiringPiSetup() == -1) {
        fprintf(stderr, "Unable to start WiringPi\n");
        return 1;
    }

    printf("Sending numbers to STM32...\n");

    // 숫자를 STM32로 전송
    while (1) {
        // 숫자를 문자열로 변환하여 전송
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%d\n", number_to_send);
        serialPuts(uart_fd, buffer); // UART로 전송

        printf("Sent: %s", buffer);

        // 1초 대기
        sleep(1);
    }

    // UART 종료
    serialClose(uart_fd);

    return 0;
}
