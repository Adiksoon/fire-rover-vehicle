# Fire Rover — BOM
> Cel: 1× robot mobilny gąsienicowy do pracy wewnątrz (ROS2, SLAM 2D LiDAR, ArUco).  
> Uwaga: tam gdzie nie ma konkretnego linku/modelu, zostawiam **WYMAGANIA** (żebyście mogli kupić zamiennik w PL/Ali bez ryzyka).  

## 0. Konwencje i założenia
- Napięcia: LiPo 3S = 9.0–12.6 V (nom. 11.1 V).
- Zasilanie logiki: 5 V (Jetson + USB/sensory).
- Sterowanie napędem: STM32 + 2× mostek H (lewy/prawy silnik).
- **Priorytet**: stabilność zasilania Jetsona (oddzielna przetwornica, filtracja, bezpiecznik).

---

## 1. COMPUTE / STORAGE (GÓRNY SEGMENT — Percepcja)
| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :-- | :--: | :-- |
| **NVIDIA Jetson Orin Nano 8GB DEV KIT** | 1 | 7–20V DC, min. 3–5A | https://www.conrad.pl/pl/p/jetson-orin-nano-nvidia-8-gb-6-x-1-5-ghz-2998506.html | 1789 | Komputer główny robota |
| **SSD M.2 2280 NVMe 500GB (PNY CS2230 lub równoważny)** | 1 | [-] | https://www.morele.net/dysk-ssd-pny-cs2230-500gb-m-2-2280-pci-e-x4-gen3-nvme-m280cs2230-500-rb-12718938/ | 311 | System + logi |
| **microSD 128–256GB (SanDisk Extreme lub równoważna)** | 1 | [-] | https://www.amazon.pl/Sandisk-Extreme-SDSQXAA-128G-GN6MA-Pami%C4%99ci-Czerwony/dp/B09X7CRKRZ/ | 110 | nośnik pomocniczy / recovery |


---

## 2. SENSORS (PERCEPCJA / ODOMETRIA)
| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :-- | :--: | :-- |
| **Kamera AR0234 2.3MP global shutter (Arducam B0429 lub równoważna)** | 1 | zasilanie przez CSI/Jetson | https://botland.com.pl/akcesoria-do-nvidia/23590-kamera-ar0234-23mpx-color-global-shutter-do-nvidia-jetson-nanoxavier-nxorin-nx-z-obudowa-arducam-b0429.html | 520 | ArUco |
| **RPLiDAR A1M8** | 1 | 5V (USB) | https://botland.com.pl/skanery-laserowe/19625-skaner-laserowy-rplidar-a1m8-r6-360-stopni-12m-seeedstudio-114992561-5904422369248.html | 380 | SLAM 2D |
| **IMU MPU-9250 (GY-9250) 9DOF** | 1 | 3.3–5V | https://kamami.pl/czujniki-6dof-9dof-10dof/557794-modmpu9250-gy-9250-modul-9dof-z-ukladem-mpu-9250-akcelerometr-magnetometr-zyroskop-5906623454754.html | ~35 | EKF / orientacja |
| **Przewód CSI/adapter (jeśli wymagany przez wybraną kamerę)** | 1 | [-] | WYMAGANIA: zgodny z Jetson Orin Nano i modułem kamery | ~30 | często w komplecie – zweryfikować |

---

## 3. MCU / CONTROL
| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :-- | :--: | :-- |
| **STM32 NUCLEO-F401RE** | 1 | 5V | https://kamami.pl/stm-nucleo-64/212018-nucleo-f401re-zestaw-startowy-z-mikrokontrolerem-z-rodziny-stm32-stm32f401-5906623435852.html | 80 | Sterowanie napędem, odometria |
| **Konwerter poziomów logicznych (jeśli IMU 3.3V a I2C 5V)** | 1 | 3.3↔5V | WYMAGANIA: bidirectional I2C level shifter | ~10 | zależnie od podłączenia IMU |

---

## 4. DRIVE / ACTUATION (NAPĘD)
| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :-- | :--: | :-- |
| **Tank Chassis MG540 (z enkoderami)** | 1 | 12V | https://pl.aliexpress.com/item/1005008489785257.html | 520 | podwozie |
| **Sterownik silnika BTS7960** | 2 | 6–27V | https://botland.com.pl/sterowniki-silnikow/2503-sterownik-silnika-bts7960-43a-5904422334437.html | ~35 | 1× lewy + 1× prawy |
| **Filtr/odsprzęganie przy mostkach H (kondensator elektrolityczny)** | 2 | ≥25V | WYMAGANIA: 2200 µF 25V low-ESR + złącze/termokurcz | ~10 | montować blisko BTS |
| **Kondensator przy wejściu Jetsona (bufor)** | 1 | ≥16V | WYMAGANIA: 1000 µF 16V low-ESR | ~5 | przy jacku zasilania Jetsona |

---

## 5. POWER (ZASILANIE)
| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :-- | :--: | :-- |
| **LiPo 3S 5000mAh 60C (XT60)** | 1 | 11.1V | https://www.modelmania.pl/sklep/strona-glowna/209-akumulator-gens-ace-5000mah-111v-60c-3s-xt60.html | ~150 | zasilanie robota |
| **Ładowarka LiPo (SkyRC B6 lub równoważna)** | 1 | 230V | LINK_LADOWARKA | 230 | ładowanie |
| **Wyłącznik główny zasilania** | 1 | ≥20A DC | WYMAGANIA: DC switch min. 20A, montaż panelowy | ~10–30 | odcina cały robot |
| **Bezpiecznik główny + holder (przy baterii)** | 1 | 20–30A | WYMAGANIA: holder + bezpiecznik ATO/ANL 20–30A | ~20 | dobór po pomiarach prądu |
| **Skrzynia bezpiecznikowa / dystrybucja** | 1 | 12V | https://pl.aliexpress.com/item/1005008090414736.html | 30 | rozdział gałęzi |
| **DC-DC: 5V 8–10A (TYLKO dla Jetsona)** | 1 | 7–24V → 5V | WYMAGANIA: buck 5V ≥8A, niski ripple, radiator | ~60–120 | krytyczne: stabilność Jetsona |
| **DC-DC: 5V 3A (STM32 + IMU + akcesoria)** | 1 | 7–24V → 5V | WYMAGANIA: buck 5V ≥3A | ~20–40 | osobna gałąź „clean” |
| **LiPo voltage alarm (buzzer)** | 1 | [-] | https://pl.aliexpress.com/item/1005002718956934.html | 5 | jako backup, nie zastępuje pomiaru w ROS |
| **Pomiar napięcia baterii do STM32 (dzielnik rezystorowy)** | 1 kpl. | [-] | WYMAGANIA: rezystory np. 100k + 33k (dobór do ADC) | ~2 | telemetria baterii |
| **E‑STOP (grzybek, NC, odcina zasilanie driverów)** | 1 | ≥20A DC | WYMAGANIA: Emergency stop, styk NC, panel | ~30–60 | strongly recommended |

---

## 6. CONNECTIVITY / CABLES (KABLE / POŁĄCZENIA)
| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :-- | :--: | :-- |
| **Wtyk zasilający Jetson 5.5/2.5mm** | 1 | [-] | https://pl.aliexpress.com/item/1005007656068465.html | ~5 | do kabla z buck 5V |
| **USB A → mini USB 0.5m (STM32 ↔ Jetson)** | 1 | [-] | https://pl.aliexpress.com/item/1005002253497663.html | ~4 | jeśli zostajecie na USB |
| **Ethernet CAT6 2m (debug)** | 1 | [-] | https://pl.aliexpress.com/item/1005009121518528.html | ~8 | Jetson ↔ PC |
| **Powered USB Hub (zasilany)** | 1 | 5V | WYMAGANIA: aktywny hub 4-port, zasilacz/5V in | ~40–80 | zwykle potrzebny (LiDAR + debug) |
| **Przewody silikonowe do silników (15AWG)** | 2–3 m | [-] | LINK_WIRE | ~20/m | zasilanie napędu, skręcać pary |
| **Przewody sygnałowe (22–26AWG)** | 5–10 m | [-] | WYMAGANIA: przewód wielożyłowy, elastyczny | ~20–40 | enkodery, I2C, GPIO |
| **Złącza JST‑XH / JST‑PH (zestaw)** | 1 | [-] | WYMAGANIA: komplet wtyk+gniazdo + piny + zaciskarka (opcjonalnie) | ~30–80 | bardzo ułatwia porządek w kablach |
| **Koszulki termokurczliwe (zestaw)** | 1 | [-] | lokalnie | ~10–20 | izolacja |
| **Opaski zaciskowe (trytytki) + rzepy** | 1 kpl. | [-] | lokalnie | ~10–20 | organizacja przewodów |
| **Ferryt na przewody / rdzenie ferrytowe (zestaw)** | 1 | [-] | WYMAGANIA: klipsy ferrytowe na przewody zasilania/USB | ~20–40 | redukcja EMI |
| **Zaciski oczkowe + końcówki tulejkowe + konektory** | 1 kpl. | [-] | lokalnie | ~20–50 | połączenia zasilania |

---

## 7. MECHANICAL / MOUNTING (MONTAŻ)
| MODEL | ILOŚĆ | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :--: | :-- |
| **Dystanse montażowe M3** | 1 zestaw | LINK_STANDOFF | ~20 | montaż elektroniki |
| **Śruby M3 (zestaw)** | 1 zestaw | LINK_SCREWS | ~20 | montaż |
| **Mocowanie LiDAR (uchwyt) + śruby** | 1 | WYMAGANIA: sztywne mocowanie, minimalne drgania | ~20 | można druk 3D |
| **Mocowanie kamery (uchwyt) + śruby** | 1 | WYMAGANIA: regulacja kąta | ~20 | można druk 3D |
| **Taśma dwustronna piankowa / podkładki antywibracyjne** | 1 | lokalnie | ~10–20 | pod IMU / elektronikę |

---

## 8. SPARES / SERWIS (opcjonalne, ale polecam)
| MODEL | ILOŚĆ | UWAGI |
| :-- | :--: | :-- |
| Zapasowe bezpieczniki (ATO/ANL) | 1 kpl. | zawsze schodzą na testach |
| Zapasowy BTS7960 | 1 | przy błędach okablowania potrafi paść |
| Zapas przewodu 15AWG + 22AWG | 1 kpl. | naprawy w terenie |

---

## 9. TODO — pola do uzupełnienia linkami
- LINK_LADOWARKA
- LINK_PRZETWORNICA (zastąpione przez 2 przetwornice: 5V 10A + 5V 3A)
- LINK_WIRE
- LINK_STANDOFF
- LINK_SCREWS

---

## 10. Notatki integracyjne (krótko)
- **Jetson** zasilać z osobnej przetwornicy 5V ≥8A + bezpiecznik na gałęzi.
- Przewody silników skręcać parami, prowadzić z dala od USB/LiDAR.
- Kondensatory 2200 µF montować możliwie blisko BTS7960.
