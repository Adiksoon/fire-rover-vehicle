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


---

## 3. MCU / CONTROL
| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :-- | :--: | :-- |
| **STM32 NUCLEO-F401RE** | 1 | 5V | https://kamami.pl/stm-nucleo-64/212018-nucleo-f401re-zestaw-startowy-z-mikrokontrolerem-z-rodziny-stm32-stm32f401-5906623435852.html | 80 | Sterowanie napędem, odometria |
| **Konwerter poziomów logicznych (jeśli IMU 3.3V a I2C 5V)** | 1 | 3.3↔5V | https://www.sparkfun.com/sparkfun-logic-level-converter-bi-directional.html | ~10 | zależnie od podłączenia IMU |

---

## 4. DRIVE / ACTUATION (NAPĘD)
| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :-- | :--: | :-- |
| **Tank Chassis MG540 (z enkoderami)** | 1 | 12V | https://pl.aliexpress.com/item/1005008489785257.html | 520 | podwozie |
| **Sterownik silnika BTS7960** | 2 | 6–27V | https://botland.com.pl/sterowniki-silnikow/2503-sterownik-silnika-bts7960-43a-5904422334437.html | ~35 | 1× lewy + 1× prawy |
| **Filtr/odsprzęganie przy mostkach H (kondensator elektrolityczny)** | 2 | ≥25V | https://www.tme.eu/pl/details/eeufr1e222l/kondensatory-elektrolityczne-tht/panasonic/ | ~10 | montować blisko BTS |


---

## 5. POWER (ZASILANIE)
| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :-- | :--: | :-- |
| **LiPo 3S 5000mAh 60C (XT60)** | 1 | 11.1V | https://www.modelmania.pl/sklep/strona-glowna/209-akumulator-gens-ace-5000mah-111v-60c-3s-xt60.html | ~150 | zasilanie robota |
| **Ładowarka LiPo (SkyRC B6 lub równoważna)** | 1 | 230V | https://botland.com.pl/ladowarki-lipo/6103-ladowarka-li-polli-hvli-ionli-feni-cdni-mh-z-balanserem-skyrc-imax-b6ac-v2-usb-z-wbudowanym-zasilaczem-6930460000040.html | 230 | ładowanie |
| **Wyłącznik główny zasilania** | 1 | ≥20A DC | https://www.tme.eu/pl/details/rb2-1a-dc-2-rl/przelaczniki-typu-rocker/switch-components/ | ~10–30 | odcina cały robot |
| **Bezpiecznik główny + holder (przy baterii)** | 1 | 20–30A | https://sklep.autokran.pl/hiab/1379-bezpiecznik-automat-automatyczny-samochodowy-ato-30a.html https://www.tme.eu/pl/details/mta-uni-holder/gniazda-bezpiecznikowe-samochodowe/mta/0300336/?brutto=1&currency=PLN&utm_source=google&utm_medium=cpc&utm_campaign=Elektromechaniczne%20PL%20%5BPLA%5D%20CSS&utm_content=&campaign_id=11197053306&gad_source=1&gad_campaignid=11197053306&gbraid=0AAAAADyylhJljpRwkNzKDfYvivmLRVDRB&gclid=Cj0KCQiAk6rNBhCxARIsAN5mQLsgg1sk5pOx_kMx62mCaqNBA3mcZ9W_lBef7fpm1cKWabeNtoZUmrEaAuv4EALw_wcB | ~20 | dobór po pomiarach prądu |
| **Skrzynia bezpiecznikowa / dystrybucja** | 1 | 12V | https://pl.aliexpress.com/item/1005008090414736.html | 30 | rozdział gałęzi |
| **DC-DC: 5V 8–10A (TYLKO dla Jetsona)** | 1 | 7–24V → 5V | https://www.pololu.com/product/2866/specs | ~60–120 | krytyczne: stabilność Jetsona |
| **DC-DC: 5V 3A (STM32 + IMU + akcesoria)** | 1 | 7–24V → 5V | https://sklep.msalamon.pl/produkt/przetwornica-3a-dc-dc-step-down-lm2596/?gad_source=1&gad_campaignid=21700502614&gbraid=0AAAAAByhW9BERkisyFbXIHTVkC8GZZTcs&gclid=Cj0KCQiAk6rNBhCxARIsAN5mQLvXmPejkDLbfiiq1ts0b3mkgYID4kcBHriv6J9iyusnVphqhNKcJlsaAq5NEALw_wcB | ~20–40 | osobna gałąź „clean” |
| **LiPo voltage alarm (buzzer)** | 1 | [-] | https://pl.aliexpress.com/item/1005002718956934.html | 5 | jako backup, nie zastępuje pomiaru w ROS |
| **Pomiar napięcia baterii do STM32 (dzielnik rezystorowy)** | 1 kpl. | [-] | https://pl.farnell.com/yageo/mfr-25frf52-33k/res-33k-1-metal-film-axial/dp/4560130?CMP=KNC-GPL-GEN-SKU-Optmyzr&mckv=s_dc|pcrid|763752460078|kword|mfr-25frf52-33k|match|p|plid||slid||product||pgrid|182081268429|ptaid|kwd-833203168987|&gad_source=1&gad_campaignid=22787273751&gbraid=0AAAAAD8yeHm73X9qef1Xal9oDUNLqwLPo&gclid=Cj0KCQiAk6rNBhCxARIsAN5mQLtB1U0YoSkDM-lEXGPd8MoMLp8SefGFlMMJ1AqU6kvWX6U_K3UUpfUaAuPDEALw_wcB https://pl.farnell.com/yageo/mfr-25fte52-100k/res-100k-1-0-25w-axial-metal-film/dp/3496900 | ~2 | telemetria baterii |
| **E‑STOP (grzybek, NC, odcina zasilanie driverów)** | 1 | ≥20A DC | https://www.tme.eu/pl/details/rrjuv/przelaczniki-tablicowe-standardowe-22mm/schlegel/? | ~30–60 | strongly recommended |

---

## 6. CONNECTIVITY / CABLES (KABLE / POŁĄCZENIA)
| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :-- | :--: | :-- |
| **Wtyk zasilający Jetson 5.5/2.5mm** | 1 | [-] | https://pl.aliexpress.com/item/1005007656068465.html | ~5 | do kabla z buck 5V |
| **USB A → mini USB 0.5m (STM32 ↔ Jetson)** | 1 | [-] | https://pl.aliexpress.com/item/1005002253497663.html | ~4 | jeśli zostajecie na USB |
| **Ethernet CAT6 2m (debug)** | 1 | [-] | https://pl.aliexpress.com/item/1005009121518528.html | ~8 | Jetson ↔ PC |
| **Powered USB Hub (zasilany)** | 1 | 5V | https://allegro.pl/oferta/orico-hub-aktywny-biurkowy-4-usb-a-5gbps-aluminium-17574811803?bi_s=ads&bi_m=productlisting:desktop:query&bi_c=ZDczNGZlMzYtYTIxYi00Y2EwLWJiN2EtZDNlOGNkNTZlMjU0AA&bi_t=ape&referrer=proxy&emission_unit_id=2b27075c-cb06-4c20-8121-bea02c27bb5b | ~40–80 | zwykle potrzebny (LiDAR + debug) |
| **Przewody silikonowe do silników (15AWG)** | 2–3 m | [-] | https://www.rc4max.com/przewod-silikonowy-1-7-mm-15-awg-ok-1mb-czarny-+-czerwony | ~20/m | zasilanie napędu, skręcać pary |
| **Złącza JST‑XH / JST‑PH (zestaw)** | 1 | [-] | https://botland.com.pl/zlacza-raster-254mm/23834-zestaw-zlacz-jst-xh254-678910-pin-meskiezenskie-i-pinow-zenskich-do-obudowy-gniazda-260szt-justpi-5904422384234.html | ~30–80 | bardzo ułatwia porządek w kablach |
| **Koszulki termokurczliwe (zestaw)** | 1 | [-] | lokalnie | ~10–20 | izolacja |
| **Opaski zaciskowe (trytytki) + rzepy** | 1 kpl. | [-] | lokalnie | ~10–20 | organizacja przewodów |
| **Ferryt na przewody / rdzenie ferrytowe (zestaw)** | 1 | [-] | https://www.amazon.com/IEUYO-Ferrite-Signal-Suppressor-Diameter/dp/B07DPM44BV? | ~20–40 | redukcja EMI |
| **Zaciski oczkowe + końcówki tulejkowe + konektory** | 1 kpl. | [-] | lokalnie | ~20–50 | połączenia zasilania |

---

## 7. MECHANICAL / MOUNTING (MONTAŻ)
| MODEL | ILOŚĆ | LINK | CENA | UWAGI |
| :-- | :--: | :-- | :--: | :-- |
| **Dystanse montażowe M3** | 1 zestaw | https://botland.com.pl/tuleje-dystansowe/23074-zestaw-srubek-i-podkladek-dystansowych-m3-zestaw-a-120szt-justpi-5904422383985.html | ~20 | montaż elektroniki |
| **Śruby M3 (zestaw)** | 1 zestaw | https://botland.com.pl/tuleje-dystansowe/23074-zestaw-srubek-i-podkladek-dystansowych-m3-zestaw-a-120szt-justpi-5904422383985.html | ~20 | montaż |
| **Mocowanie LiDAR (uchwyt) + śruby** | 1 | WYMAGANIA: sztywne mocowanie, minimalne drgania | ~20 | można druk 3D |
| **Mocowanie kamery (uchwyt) + śruby** | 1 | WYMAGANIA: regulacja kąta | ~20 | można druk 3D |
| **Taśma dwustronna piankowa / podkładki antywibracyjne** | 1 | lokalnie | ~10–20 | pod IMU / elektronikę |

---

## 8. SPARES / SERWIS
| MODEL | ILOŚĆ | UWAGI |
| :-- | :--: | :-- |
| Zapasowe bezpieczniki (ATO/ANL) | 1 kpl. | zawsze schodzą na testach |
| Zapasowy BTS7960 | 1 | przy błędach okablowania potrafi paść |
| Zapas przewodu 15AWG + 22AWG | 1 kpl. | naprawy w terenie |

---


## 9. Notatki integracyjne (krótko)
- **Jetson** zasilać z osobnej przetwornicy 5V ≥8A + bezpiecznik na gałęzi.
- Przewody silników skręcać parami, prowadzić z dala od USB/LiDAR.
- Kondensatory 2200 µF montować możliwie blisko BTS7960.
