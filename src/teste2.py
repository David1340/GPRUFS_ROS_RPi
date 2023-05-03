import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise IOError("Não foi possvel abrir a câmera")

while(True):
    ret, frame = cap.read()

    if(not ret):
        break

    cv2.imshow('camera',frame)

    if(cv2.waitKey(1) & 0xFF == ord('q')):
        break

# Libere a câmera e feche a janela
cap.release()
cv2.destroyAllWindows()