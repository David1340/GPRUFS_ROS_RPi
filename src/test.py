import serial

stm32 = serial.Serial(port = '/dev/ttyACM0',baudrate=115200)
try:
    msg2 = [255,0,70]
    for _ in range(1000):
        stm32.write(msg2)

        if(stm32.inWaiting() > 1):
            msg_read = stm32.readline(3).decode().split()
            print(msg_read)

except KeyboardInterrupt:
    stm32.close()
    print("Execução encerrada pelo usuário")