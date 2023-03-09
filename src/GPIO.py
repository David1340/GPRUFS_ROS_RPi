import RPi.GPIO as gpio
import time

gpio.setmode(gpio.BOARD) #se você quer se referir aos pinos pela sua numeração
#gpio.setmide(gpio.BCM) #se você quer se referir aos pinos da mesma forma que o fabricante

led = 37 #número de pino em que está o LED
gpio.setup(led,gpio.OUT) #configura o pino led com saída
#gpio.output(led, True) $coloca o pino LED em valor lógico alto

button = 35 #número do pino em que está o botão
gpio.setup(button,gpio.IN,gpio.PUD_UP) #configurando o pino do botão como entrada com resistor pull-up
while(True):
    if not gpio.input(button):
        gpio.output(led, True)
    else:
        gpio.output(led, False)

pwm = gpio.PWM(led,1000)  #pino e frequência
duty_cicle = 0
pwm.start(duty_cicle) #inicia o pwm com um duty cicle inicial.
for duty_cicle in range(0,110,10):
    pwm.ChangeDutyCycle(100) #altera o valor do pwm
    print(duty_cicle,'\n')
    time.sleep(2)
pwm.stop() #desliga o PWM


