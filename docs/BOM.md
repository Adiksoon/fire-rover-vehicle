# GÓRNY SEGMENT — Percepcja 

| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :------------------------------------------ | :------: | :------------------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :--: | :--------------------- |
| **NVIDIA Jetson Orin Nano 8GB DEV KIT** | 1 | 7–20V DC, min. 3–5A | [link](https://www.conrad.pl/pl/p/jetson-orin-nano-nvidia-8-gb-6-x-1-5-ghz-2998506.html) | 1789 | Komputer główny robota |
| **Dysk SSD PNY CS2230 500GB** | 1 | [-] | [link](https://www.morele.net/dysk-ssd-pny-cs2230-500gb-m-2-2280-pci-e-x4-gen3-nvme-m280cs2230-500-rb-12718938/) | 311 | System + logi |
| **SanDisk Extreme 256GB** | 1 | [-] | [link](https://www.amazon.pl/Sandisk-Extreme-SDSQXAA-128G-GN6MA-Pami%C4%99ci-Czerwony/dp/B09X7CRKRZ/) | 110 | microSD |
| **Kamera AR0234 2.3MPx** | 1 | [-] | [link](https://botland.com.pl/akcesoria-do-nvidia/23590-kamera-ar0234-23mpx-color-global-shutter-do-nvidia-jetson-nanoxavier-nxorin-nx-z-obudowa-arducam-b0429.html) | 520 | Global shutter |
| **Skaner laserowy RPLiDAR A1M8** | 1 | [-] | [link](https://botland.com.pl/skanery-laserowe/19625-skaner-laserowy-rplidar-a1m8-r6-360-stopni-12m-seeedstudio-114992561-5904422369248.html) | 380 | SLAM |
| **STM32 NUCLEO-F401RE** | 1 | 5V | [link](https://kamami.pl/stm-nucleo-64/212018-nucleo-f401re-zestaw-startowy-z-mikrokontrolerem-z-rodziny-stm32-stm32f401-5906623435852.html) | 80 | Sterowanie napędem |
| **IMU MPU-9250 (GY-9250) 9DOF** | 1 | 3.3–5V | [link](https://kamami.pl/czujniki-6dof-9dof-10dof/557794-modmpu9250-gy-9250-modul-9dof-z-ukladem-mpu-9250-akcelerometr-magnetometr-zyroskop-5906623454754.html) | ~35 | IMU do orientacji/odometrii |

---

# DOLNY SEGMENT — Zasilanie / Napęd

| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :------------------------------------------ | :------: | :------------------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :--: | :--------------------- |
| **Tank Chassis MG540 (z enkoderami)** | 1 | 12V | [link](https://pl.aliexpress.com/item/1005008489785257.html) | 520 | Podwozie gąsienicowe |
| **LiPo 3S Gens Ace 5000mAh 60C** | 1 | 11.1V | [link](https://www.modelmania.pl/sklep/strona-glowna/209-akumulator-gens-ace-5000mah-111v-60c-3s-xt60.html) | ~150 | Zasilanie robota XT60 |
| **Ładowarka LiPo (np. SkyRC B6)** | 1 | 230V | [link](LINK_LADOWARKA) | 230 | Ładowanie akumulatora |
| **LiPo voltage alarm** | 1 | [-] | [link](https://pl.aliexpress.com/item/1005002718956934.html) | 5 | Alarm napięcia |
| **Wyłącznik główny zasilania** | 1 | [-] | brak | 10 | kupić lokalnie lub poszukac na ali|
| **Przetwornica DC-DC STEP DOWN 10V 5V** | 1 | 7–24V → 5V | [link](LINK_PRZETWORNICA) | ~35 | Zasilanie STM |
| **Sterownik silnika BTS7960** | 2 | 6–27V | [link](https://botland.com.pl/sterowniki-silnikow/2503-sterownik-silnika-bts7960-43a-5904422334437.html) | ~35 | Po jednym na silnik |
| **Dystanse montażowe M3** | 1 zestaw | [-] | [link](LINK_STANDOFF) | ~20 | Montaż elektroniki 
| **Śruby M3 (zestaw)** | 1 | [-] | [link](LINK_SCREWS) | ~20 | Montaż |
| **Skrzynia Bezpiecznikowa** | 1 | 12V | [link](https://pl.aliexpress.com/item/1005008090414736.html) | 30 | Dystrybucja zasilania |


---

# ELEMENTY DO KUPIENIA LOKALNIE

| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :------------------------------------------ | :------: | :------: | :--: | :--: | :--------------------- |
| **Zaciski oczkowe do przewodów zasilających** | zestaw | [-] | brak | [-] | kupić lokalnie |
| **Zaciski przewodów do silników i step-down** | zestaw | [-] | brak | [-] | kupić lokalnie |
| **XT60 przewody (męska końcówka)** | 2 | [-] | brak | ~10 | kupić lokalnie |
| **Bezpieczniki ATO** | 3 | [-] | brak | [-] | MAMY (5A Jetson, 15A silniki, 2A STEP DOWN) |
| **Holder na główny bezpiecznik** | 1 | [-] | brak | [-] | kupić lokalnie |
| **Koszulki termokurczliwe** | zestaw | [-] | brak | [-] | kupić lokalnie |

---

# KABLE / POŁĄCZENIA

| MODEL | ILOŚĆ | Napięcie | LINK | CENA | UWAGI |
| :------------------------------------------ | :------: | :------: | :------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :--: | :--------------------- |
| **Wtyk zasilający Jetson 5.5mm / 2.5mm** | 1 | [-] | [link](https://pl.aliexpress.com/item/1005007656068465.html) | ~5 | zasilanie Jetsona |
| **Ethernet CAT6 2m** | 1 | [-] | [link](https://pl.aliexpress.com/item/1005009121518528.html) | ~8 | Jetson ↔ komputer |
| **USB A → mini USB 0.5m** | 1 | [-] | [link](https://pl.aliexpress.com/item/1005002253497663.html) | ~4 | STM ↔ Jetson |
| **Przewody silikonowe 15AWG** | 1m | [-] | [link](LINK_WIRE) | ~20 | Zasilanie silników |
