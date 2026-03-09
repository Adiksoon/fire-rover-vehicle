# Architektura systemu

## Schemat blokowy

### START
**Opis:**  
Inicjalizacja systemu, uruchomienie węzłów ROS2 oraz sterowników sprzętowych.

**Wyjście:**  
- LIDAR  
- OAK-D LITE  

---

### LIDAR (Laser Scanner)
**Opis:**  
Sensor wykonujący skanowanie otoczenia w płaszczyźnie 2D. Dane geometryczne (odległość od ścian i przeszkód statycznych) są wykorzystywane do budowy mapy.

**Wyjście:**  
- SLAM 3D  

---

### OAK-D LITE
**Opis:**  
Kamera stereoskopowa generująca chmurę punktów RGB-D.  
Służy do mapowania przestrzennego, weryfikacji obiektów oraz omijania przeszkód w trzech wymiarach.

**Wyjście:**  
- SLAM 3D  
- YOLO VPU  

---

### SLAM 3D (Simultaneous Localization and Mapping)
**Opis:**  
Moduł **RTAB-Map** integrujący dane z LIDAR oraz OAK-D LITE w celu stworzenia trójwymiarowej mapy otoczenia (**OctoMap**).

**Wyjście:**  
- EXPLORATION PLANNER  
- EKF CHECK  

---

### EXPLORATION PLANNER
**Opis:**  
Moduł odpowiedzialny za autonomiczną eksplorację sali (**Frontier Exploration**).  
Wyznacza tymczasowe cele ruchu w nieodkrytych obszarach mapy, dopóki obiekt nie zostanie wykryty.

**Wyjście:**  
- CAMERA PT  

---

### CAMERA PT (Pan-Tilt Camera)
**Opis:**  
Szerokokątna kamera (**160° FOV**) na ruchomej wieżyczce odpowiedzialna za aktywne, wstępne przeszukiwanie sali niezależnie od kierunku jazdy robota.

**Wyjście:**  
- YOLO  

---

### YOLO
**Opis:**  
Proces detekcji obiektów realizowany na obrazie z kamery szerokokątnej.  
W momencie wizualnego rozpoznania celu przerywa tryb eksploracji.

**Wyjście:**  
- YOLO VPU  

---

### YOLO VPU (Verification Mode)
**Opis:**  
Moduł weryfikacji działający na koprocesorze kamery **OAK-D LITE**.  
Po otrzymaniu sygnału z YOLO wykonuje precyzyjną weryfikację obiektu i wyznacza jego współrzędne przestrzenne **goal_pose(t)**.

**Wyjście:**  
- Węzeł sumacyjny 1  

---

### EKF (Extended Kalman Filter)
**Opis:**  
Filtr dokonujący fuzji danych z:

- enkoderów
- IMU
- odometrii wizualnej z OAK-D LITE

w celu wyznaczenia stabilnej odometrii lokalnej.

**Wyjście:**  
- EKF CHECK  

---

### EKF CHECK
**Opis:**  
Węzeł korygujący dane z filtra EKF na podstawie globalnej mapy z modułu SLAM 3D.  
Wyznacza ostateczną pozycję robota w przestrzeni **robot_pose(t)**.

**Wyjście:**  
- Węzeł sumacyjny 1  

---

### Węzeł sumacyjny 1
**Opis:**  
Oblicza uchyb poprzez porównanie aktualnej pozycji **robot_pose(t)** z pozycją celu **goal_pose(t)**.

**Wyjście:**  
- ROUTE PLANNER  

---

### ROUTE PLANNER (Global Planner)
**Opis:**  
Planowanie optymalnej trajektorii do celu na podstawie mapy 3D, uwzględniające najkorzystniejszą trasę przejazdu.

**Wyjście:**  
- REAL TIME PLANNER  

---

### REAL TIME PLANNER (Local Planner)
**Opis:**  
Moduł korygujący trajektorię w czasie rzeczywistym.  
Wykorzystuje chmurę punktów z **OAK-D LITE** do wykrywania i omijania przeszkód niewidocznych dla LIDAR-u.

**Wyjście:**  
- PID  

---

### PID (Controller)
**Opis:**  
Regulator sterowania obliczający sygnały prędkości zadanej na podstawie wyznaczonego uchybu trajektorii.

**Wyjście:**  
- MOTORS  

---

### MOTORS (Drive System)
**Opis:**  
Układ wykonawczy robota (sterowniki i silniki gąsienic), wprawiający platformę w ruch.

**Wyjście:**  
- ROBOT  

---

### ROBOT (Mobile Robot Platform)
**Opis:**  
Fizyczny obiekt sterowania.  
Ruch robota jest monitorowany przez czujniki wewnętrzne.

**Wyjście:**  
- IMU  
- ENCODERS  

---

### IMU & ENCODERS
**Opis:**  
Czujniki inercyjne oraz enkodery kół mierzące parametry ruchu fizycznego.  
Dane trafiają do filtra EKF, zamykając pętlę sprzężenia zwrotnego pozycji.

**Wyjście:**  
- EKF  
