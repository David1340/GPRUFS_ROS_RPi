from std_msgs.msg import Float32MultiArray

x = Float32MultiArray()
x.data = [2.1,2.3]
print(x)

from sensor_msgs.msg import Image
i = Image()
print(i)
import cv2
import numpy as np

frame = np.zeros((1024,1024,3),np.uint8)
cv2.rectangle(frame, (400, 400), (200, 200), (0,0,255), -1)
cv2.imshow('Imagem', frame)
cv2.waitKey(0) # Aguarda até que uma tecla seja pressionada para fechar a janela
cv2.destroyAllWindows() # Fecha todas as janelas abertas
