# Architektura systemu

## 1. Percepcja
Przetwarzanie obrazu z kamery i wykrywanie znacznika ArUco.

## 2. Lokalizacja
Budowa mapy otoczenia i estymacja aktualnej pozycji robota na mapie.

## 3. Nawigacja
Wyznaczanie trasy do punktu docelowego oraz lokalne omijanie przeszkod.

## 4. Sterowanie
Realizacja polecen predkosci i translacja komend ruchu na naped robota.

## 5. Komunikacja
Wymiana danych miedzy komputerem pokladowym, sterownikami silnikow i enkoderami.

**Schemat blokowy**

**START** – inicjalizacja systemu oraz rozpoczęcie pracy robota.
Wyjście: **LIDAR**.

**LIDAR (Laser Scanner)** – sensor analizujący otoczenie robota i generujący chmurę punktów opisującą środowisko. Dane te wykorzystywane są zarówno do mapowania otoczenia, jak i do wykrywania przeszkód w czasie rzeczywistym.
Wyjście: **LOCAL PLANNER** oraz **SLAM**.

**SLAM (Simultaneous Localization and Mapping)** – moduł odpowiedzialny za jednoczesne tworzenie mapy środowiska oraz estymację położenia robota w przestrzeni. Na podstawie chmury punktów z lidaru tworzona jest mapa otoczenia (**/map**).
Wyjście: **ROUTE PLANNER**, **węzeł sumacyjny 1**, **SEARCH**.

**ENCODERS / ODOMETRY (/odom)** – moduł odometrii wyznaczający przybliżone przemieszczenie robota na podstawie prędkości i obrotów kół mierzonych przez enkodery.
Wyjście: **węzeł sumacyjny 1**.

**Węzeł sumacyjny 1** – łączy informacje o pozycji robota pochodzące z odometrii (**/odom**) oraz z lokalizacji SLAM (**/map**). Na tej podstawie wyznaczana jest aktualna pozycja robota w przestrzeni, oznaczona jako **robot_pose(t)**.
Wyjście: **ROUTE PLANNER**, **LOCAL PLANNER**, **węzeł sumacyjny 2**.

**SEARCH (Exploration Mode)** – tryb eksploracji środowiska, w którym robot porusza się po pomieszczeniu do momentu wykrycia znacznika ArUco.
Wyjście: FOUND : **CAMERA / ARUCO DETECTION** NOT FOUND : **LOCAL_PLANNER**.

**CAMERA / ARUCO DETECTION** – moduł przetwarzania obrazu z kamery, odpowiedzialny za wykrycie znacznika ArUco. Na podstawie analizy obrazu wyznaczana jest pozycja celu **goal_pose(t)**.
Wyjście: **ROUTE PLANNER**.

**ROUTE PLANNER (Global Planner)** – globalny planner trajektorii. Na podstawie mapy środowiska (**/map**), aktualnej pozycji robota (**robot_pose**) oraz pozycji celu (**goal_pose**) wyznaczana jest globalna trajektoria prowadząca do celu. Analizowana jest opłacalność różnych tras, a następnie wybierana jest trajektoria najbardziej korzystna.
Wyjście: **LOCAL PLANNER**.

**LOCAL PLANNER (Local Trajectory Planner)** – moduł lokalnego planowania trajektorii. Dokonuje korekty trajektorii w czasie rzeczywistym, tak aby robot mógł omijać przeszkody i reagować na zmiany w otoczeniu. Wykorzystuje dane z lidaru (wykrywanie przeszkód), aktualną pozycję robota (**robot_pose**) oraz globalną trajektorię wyznaczoną przez route planner.
Wyjście: **węzeł sumacyjny 2**.

**Węzeł sumacyjny 2** – oblicza różnicę pomiędzy aktualną pozycją robota (**robot_pose**) a referencyjną trajektorią wyznaczoną przez local planner. Wyznaczony uchyb przekazywany jest do regulatora.
Wyjście: **REGULATOR GC1**.

**REGULATOR GC1 (Controller)** – regulator sterowania, który na podstawie wyznaczonego uchybu oblicza sygnały sterujące dla układu napędowego robota.
Wyjście: **SERVO & MOTORS**.

**SERVO & MOTORS (Drive System)** – układ wykonawczy robota. Otrzymane sygnały sterujące powodują odpowiedni ruch napędów robota.
Wyjście: **ROBOT**.

**ROBOT (Plant / Mobile Robot Platform)** – fizyczny obiekt sterowania. Na jego wyjściu dostępna jest informacja o prędkości i ruchu robota, która jest wykorzystywana przez enkodery do wyznaczenia odometrii.

**ENCODERS** – czujniki mierzące prędkość obrotową kół robota. Dane te wykorzystywane są do obliczenia odometrii (**/odom**), która trafia ponownie do **węzła sumacyjnego 1**, zamykając pętlę estymacji położenia robota.

