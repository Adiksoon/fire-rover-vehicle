# Autonomous Tracked Vehicle

## 1. Cel pracy
Celem pracy jest oprogramowanie i uruchomienie autonomicznego pojazdu mobilnego na platformie gąsienicowej. Głównym zadaniem robota jest realizacja fuzji danych sensorycznych (LiDAR + kamera stereoskopowa) do budowy trójwymiarowej mapy otoczenia (3D SLAM). Na podstawie tej mapy system ma planować trajektorię i dotrzeć do celu, którym jest fizyczny obiekt (np. gaśnica, plecak, krzesło) rozpoznawany w czasie rzeczywistym przez sieć neuronową (np. YOLO) uruchomioną na akceleratorze sprzętowym kamery.

## 2. Hardware
Konfiguracja opiera się na gotowej platformie badawczej z modyfikacjami zasilania i sensoryki:
- **Platforma bazowa:** UGV Beast PT ROS2 Kit (podwozie gąsienicowe, napęd z enkoderami, wieżyczka Pan-Tilt do skanowania wizyjnego).
- **Jednostka obliczeniowa:** NVIDIA Jetson Orin Nano 8GB (główny węzeł dla algorytmów RTAB-Map i planowania ścieżki).
- **Sensor wizyjny i AI:** Kamera OAK-D Lite. Odpowiada za dostarczanie chmury punktów RGB-D oraz sprzętową inferencję modelu sieci neuronowej (Spatial Object Detection) z użyciem wbudowanego koprocesora VPU.
- **Sensor dystansu:** LiDAR DToF STL27L (skanowanie 360°, zasięg 25m) – dostarcza precyzyjne pomiary odległości do fuzji z obrazem z kamery.
- **Zasilanie:** Pakiet ogniw Li-Ion (Samsung 30Q 15A) z wbudowanym układem sprzętowego UPS.

## 3. Software
Stos technologiczny w środowisku ROS2:
- **OS / Środowisko:** Ubuntu 22.04 + ROS2 Humble.
- **Fuzja Danych i Mapowanie (3D SLAM):** Pakiet RTAB-Map łączący precyzję geometrii z lasera z teksturami i głębią z kamery do budowy przestrzennej mapy zajętości (OctoMap).
- **Nawigacja:** Nawigacja uwzględniająca przeszkody statyczne i podwieszone (Local Planner zasilany na bieżąco chmurą punktów 3D).
- **Wizja Maszynowa (AI):** Detekcja obiektów oparta na modelu YOLOv8 (lub pokrewnym). Model działa wyłącznie na koprocesorze kamery (VPU), który bezpośrednio zwraca do środowiska ROS2 fizyczne współrzędne (X, Y, Z) zidentyfikowanego obiektu względem bazy robota, odciążając główny procesor.

## 4. Działanie systemu
### Start - Eksploracja przestrzeni (Frontier Exploration)
Po uruchomieniu robot zaczyna proces budowy mapy 3D (RTAB-Map). Ponieważ cel nie jest znany z góry, system wchodzi w tryb aktywnej eksploracji. Algorytm nawigacyjny wyznacza tymczasowe punkty trasy w nieodkrytych rejonach sali, zmuszając robota do fizycznego poruszania się i skanowania środowiska. W trakcie jazdy koprocesor VPU w kamerze nieprzerwanie analizuje strumień wideo pod kątem obecności poszukiwanego obiektu (YOLO).
### Wykrycie celu i planowanie (Global & Local)
W momencie zidentyfikowania obiektu przez sieć neuronową, tryb eksploracji zostaje natychmiast przerwany. Kamera sprzętowo rzutuje środek wykrytego obiektu na chmurę punktów, a jego fizyczne współrzędne (X, Y, Z) są zapisywane jako ostateczny punkt docelowy (goal_pose) na mapie. Od tego momentu Global Planner wyznacza optymalną trasę dojazdu. Podczas jazdy Local Planner w czasie rzeczywistym przetwarza dane o głębi z OAK-D Lite, korygując wektor ruchu, aby płynnie wyminąć statyczne i nowo pojawiające się przeszkody.
### Koniec
Pojazd dociera do rozpoznanego obiektu, ustawia się w zdefiniowanej strefie buforowej przodem do niego i raportuje zakończenie misji.



### Ograniczenia projektu
Konstrukcja przeznaczona wyłącznie do pracy wewnątrz budynków (indoor) na twardym i płaskim podłożu. Algorytmika nie zakłada nawigacji w terenie otwartym ani odporności na skrajne warunki oświetleniowe (np. silne odblaski, całkowite zaciemnienie), które mogłyby uniemożliwić prawidłową inferencję modelu wizyjnego.
