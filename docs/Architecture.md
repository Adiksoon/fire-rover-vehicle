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

#### STRUKTURA PAKIETOW I NODE'OW

### Robot Calibration Pakiet
|-Initialization_Node - odpowiedzialny za inicjalizacje cyklu; zaweira flage (maszyne stanu) odnosnie stanu pracy maszyny; informacje na temat szukanego modelu PUB: /Target_Config

### Robot Perception Pakiet 
Odpowiada za detekcje obrazu oraz estymacje jego polozenia wzgledem robota. System percepcji wykorzystuje dwustopniowa architekture detekcji tzn. korzystamy z dwoch kamer o roznym charakterze.
1) Kamera szerokokatna zamontowana na mechanizmie **pan-tilt** -odpowiada za wyszukiwanie obiektu
2) Kamera stereowizyjna **OAK-D Lite** - odpowiedzialna za dokladna estymacje pozycji obiektu w przestrzeni 3D. Tzn. kamera OAK-D Lite posiada wbudowany koprocesor, ktory umozliwia wykonaie inferencji sieci neuronowych bezposrednio na urzadzeniu oraz laczenie wynikow detekcji z informacja o glebi. Dzieki temu kamera potrafi zwrocic wspolrzedne przestrzenne wykrytego obiektu bez konecznosci dodatkowego przetwarzania.

## SUB: /target_config & /pan_tilt_state |-Pan_Tilt_Controller_Node - jest odpowiedzialny za sterowanie kamera w taki sposob aby jak najdokladniej przeszukiwala eksplorowany obszar PUB: /pan_tilt_cmd oraz SUB: /pan_tilt_state dzieki czemu ma pamiec odnosnie swojej pozycji.
--

##SUB: /target_config & /image_raw |-Search_Detector_Node - odpowiada za przetworzenie przez YOLO zdjec przeslanych przez kamere szerokokatna oraz wystawia flage czy taki obiekt zostal znaleziony czy nie; jesli tak publikuje informacje na temat polozenia pod jakim katem znajduje sie obiekt wzgledem robota PUB: /Search_Detection
--

## |-SearchCAM_Driver_Node - odpowiada za odebranie danych z kamery i upublikowanie ich w PUB: /image_raw
--

## SUB: /target_config |-OakD_Detector_Node - odpowiada za przetworzenie przez YOLO zdjec przeslanych przez kamere OAKD Lite; w momencie wykrycia obiektu publikuje jego pozycje PUB: /Target_Pose oraz przesyla dane z kamery w celu wizualnej odometrii PUB: /depth_rgb
--

## |-OakD_Driver_Node - odpowiada za odebranie danych z kamery i upublikowanie ich w PUB: /oakd
--
### Robot Localization
Odpowiada za estymacje aktualnej pozycji robota w czasie rzeczywistym. W systemie zastosowano filtr rozszerzonego Kalmana (EKF). Do estymacji pozycji robota wykorzystane sa trzy zrodla:
-odometria kol robota (wheel odometry)
-wizualna odometria z kamery OAK-D Lite (visual odometry)
-dane z jednostki inercyjnej IMU

--
## SUB: /wheel_speed |-Wheel_Odometry_node - node odpowiada za obliczenie odometrii robota na podstawie odczytow z enkoderow kol PUB: /wheel_odom **Typ wiadomosci**: nav_msgs/Odometry **wiadomosc zawiera** : pozycje robota; predkosc liniowa; predkosc katowa;
--
## SUB: /depth_rgb |-Visual_Odometry_node - node odpowiada za estymacje ruchu robota na podstawie danych z kamery **OAK-D Lite**. Wykorzystuje kolejne klatki obrazu oraz informacje o glebi do wyznaczenia przemieszczania kamery w czasie PUB: /visual_odom **Typ wiadomosci**: nav_msgs/Odometry **wiadomosc zawiera** : estymowana pozycje robota, orientacje, predkosc i macierz kowariancji

--
## /imu_raw |-LMU_Node - odpowiada za publikacje danych z jednostki inercyjnej. PUB: /imu **Typ wiadomosci** sensor_msgs/Imu **wiadomosc zawiera**: orientacje, przyspieszenie liniowe i predkosci katowe

--
## |- IMU_Driver_Node -  odpowiada za odebranie danych z czujnika i upublikowanie ich w PUB: /imu_raw

--
## SUB /wheel_odom & /visual_odom & /imu |-EKF_Node - node implementacji rozszerzonego filtru Kalmana laczy wszystkie zrodla odometrii. PUB: /odom **Typ wiadomosci** nav_msgs/Odometry 

### Robot_slam
Pakiet odpowiedzialny za zbudowanie SLAM Simultaneous Localization and Mapping


|-Lidar_Driver_Node - NOde odpowiada za komunikacje z LIDAREM PUB: /scan **Typ wiadomosci** sensor_msgs/LaserScan !scan zawiera informacje o odleglosci od preszkod
SUB: /scan & /odom |-Slam_Node node implementuje algorytm SLAM. Przewidywujemy wykorzystanie pakietu slam_toolbox PUB: /map **Typ wiadomosci** nav_msgs/OccupancyGrid; mapa reprezentowana jest jako siatka zajetosci 0 - wolna przestrzen 100 - przeszkoda -1 - obszar nieznany; system SLAM generuje transformacje: map -> odom natomiast pakiet robot_localization publikuje transformacje odom->base link dzieki temu powstaje standardowe drzewo transformacji ROS:
map->odom->base_link


### Robot Navigation
Pakiet odpowiada za planowanie trajektorii robota oraz generowanie polecen ruchu umozliwiajacych autonomiczne dotarcie do wyznaczonego celu. Planujemy do tego wykorzystac framework Nav2 - oficjalny system nawigacji w ROS2. Nav2 dostarcza:
- Planowanie globalnej trasy robota
- Lokalne sterowanie ruchem robota
- Unikanie przeszkod
- Budowe map kosztow
- Zarzadzanie procesem nawigacji
--
## SUB: /map & /odom & /target_pose |-Planner_Server_Node - wykorzystuje algorytmy planowania sciezki (decyzja do podjecia A*/ Dijkstra/ Smac Planner) PUB:/global_path

--
## SUB: /global_path & /odom & /costmap & /scan |- Local_Planner_Node - node odpowiada za lokalne sterowanie robotem oraz realizacje raplanowanej trajektorii PUB: /cmd_vel **Typ wiadomosci** geometry_msgs/Twist ; wykorzystuje algorytmy (decyzja do podjecia DWB/ MPPI).

## SUB: /map /scan |-Cost_Map_Node - Costmap jest mapa kosztow, ktora reprezentuje trudnosc pokonania trasy ze wzgledu na otoczenie

Dodatkowy node nie uwzgledniony we schemacie to bt_navigator jest to node z nav2 on zarzadza calym procesem nawigacji jeszcze do rozpracowania.

### Robot Control
Pakiet 'robot_control' odpowiada za sterowanie ruchem robota na podstawie polecen generowanych przez system nawigacji. Jego zadaniem jest przeksztalcenie predkosci zadanej robota w postaci komunikatu '/cmd_vel' na podstawie predkosci poszczegolnych kol robota. Sterowanie ruchem jest przy uzyciu regulatora PID ktory umozliwia podazanie robota za zadana trajektoria.

--
## SUB: /cmd_vel & /odom & /allign_vel |-Robot_Control_Node odpowiada za przeksztalcenie polecen ruchu robota w predkosci poszczegolnych kol. **Typ wiadomosci** /geometry_msgs/Twist; komunikat zawiera: linear.x -predkosc liniowa robota; angular.z - predkosc katowa robota na podstawie tych wartosci oraz modelu kinematycznego robota node oblicza zadana predkosc kol i PUB: /ref_velocity komunikat zawiera predkosc lewego i prawego kola napedowego;

--
## SUB: /scan & /wheel_speed |-Target_Alignment_Node - w momencie gdy target zostanie znaleziony przez kamere szerokokatna flaga zmieni TRUE w /Search_Detection przez to zeby uchwycila go kamera oak-d light musi zostac robot ustawiony prostopadle do obiektu; gdy flaga sie zmieni node ma za zadanie zeby ustawic robota prostopadle tak zeby obiekt znalazl sie w kadrze kamery stereoskopowej. PUB:/allign_vel do Robot_COntrol_Node ktory ma dostep takze do odometrii dlatego przy manewrze nadal bedzie uwazal na otoczenie.


### Robot Hardware Interface
Pakiet jest odpowiedzialny za komunikacje z fizycznymi komponentami robota.

--
## |-IMU_Driver_Node PUB: /imu_raw **Typ wiadomosci** sensor_msgs/Imu **Wiadomosc zawiera:** predkosci katowe przyspieszenie liniowe i orientacje

--
## |-Encoder-Driver_Node - node odczytuje dane z enkoderow zamontowanych na kolach robota PUB:/wheel_speed Topic zawiera aktualna predkosc obrotwa kol wyznaczona na podstawie impulsow enkodera

--
## SUB: /set_velocity & /wheel_speed & /imu_raw |-Motor_Controller_Node odpowaida za sterowanie predkoscia kol robota w czasie rzeczywistym PUB: wheel_velocity_cmd

--
## SUB: /weel_velocity_cmd |-Motor_Driver_Node


