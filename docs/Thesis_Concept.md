# Autonomous Tracked Vehicle

## 1. Cel pracy
Celem pracy jest zaprojektowanie i zbudowanie autonomicznego pojazdu moblinego poruszajacego sie na podwoziu gasienicowym. Robot jest zdolny do mapowania nieznanego otoczenia oraz samodzielnego dojazdu do wskazanego celu w postaci wizualnego znacznika (marker ArUco) wykrywanego przez kamerę. Trasa przejazdu wyznaczana jest automatycznie na podstawie mapy otoczenia oraz algorytmu planowania ścieżki, z uwzględnieniem przeszkód.

## 2. Hardware
Zakres pracy obejmuje:

### Część sprzętowa
- budowa pojazdu gąsienicowego,
- dobór napędu i zasilania,
- integracja komputera pokładowego,
- montaż kamery i czujników odległości.

## 3. Software
### Część programowa
- konfiguracja środowiska ROS2,
- implementacja mapowania (SLAM),
- lokalizacja robota,
- planowanie trasy przejazdu,
- detekcja celu przy użyciu kamery,
- autonomiczne sterowanie ruchem robota.



### Narzędzia i technologie
- Linux Ubuntu 22.04
- ROS2
- NVIDIA Jetson
- OpenCV
- algorytmy SLAM
- planowanie ścieżki (path planning)

## 4. Dzialanie systemu
### Start - Lokalizacja celu
Robot po uruchomieniu dokonuje skanowania otoczenia i tworzy mape otoczenia. Po wykryciu markera określana jest jego pozycja względem robota oraz wyznaczany jest punkt celu na mapie. Następnie system planowania trasy wyznacza bezkolizyjną ścieżkę przejazdu z uwzględnieniem przeszkód znajdujących się w otoczeniu.
### W trakcie dzialania
Robot na bieżąco estymuje swoją pozycję na podstawie odometrii i danych z czujników. System sterowania porównuje aktualną pozycję z zaplanowaną trajektorią i koryguje ruch robota. W przypadku pojawienia się przeszkód planowana jest lokalna korekta trasy.
### Koniec
Robot dojezdza i zatrzymuje sie przy celu

### Ograniczenia projektu
Robot przeznaczony jest do pracy w środowisku wewnętrznym (np. sala, korytarz, hala) o utwardzonej nawierzchni. Konstrukcja nie jest przystosowana do pracy na zewnątrz ani w trudnych warunkach terenowych.