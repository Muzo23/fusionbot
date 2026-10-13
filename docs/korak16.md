# Korak 16 – konfiguracija B: PROX-LOC (senzori blizine ispravljaju pozu)

**Zahtjev zadatka:** fuzija podataka iz više izvora (enkoderi + senzori blizine), robusnost na
nesigurnost i dinamičko okruženje (odbacivanje mjerenja prepreka).

## Pretpostavka (mora se navesti u radu)

Raspored labirinta je nepoznat, ali **pravila gradnje** su poznata, kao u micromouse natjecanjima:
hodnici su poravnati s osima i leže na kvadratnoj mreži poznatog koraka (1,085 m), a poznata je i
startna ćelija. Iz toga slijedi da su središnje linije hodnika i središta raskrižja na poznatim
linijama mreže. Konfiguracija D (LiDAR) tu pretpostavku neće koristiti, nego vlastitu kartu.

## Korekcije (`localization/prox_loc.py`)

1. **Hodnik** (10 Hz, dok robot vozi ravnim hodnikom): model hodnika iz koraka 8a (četiri bočna
   senzora; ili jedan zid) daje bočni pomak y od središnje linije i kut α prema osi hodnika.
   * os φ = višekratnik 90° najbliži (θ − α); središnja linija = najbliža linija mreže;
   * z = [y, α], h(x) = [n·p − c_linije, θ − φ]; R iz šuma senzora (2 mm + 1 %) prenesenog kroz
     linearni model + dno modela (4 mm, 0,5°); jedan zid × 2.
2. **Raskrižje** (na svakom događaju FSM-a): središte raskrižja je D_kanonski ispred robota i na čvoru
   mreže → koordinata robota **duž** hodnika. Preskače se ako je najbliži čvor dalje od 0,2 m (START).
3. Obje idu kroz χ² gate (99 %) EKF-a.

## Rezultati offline (7 labirinata × 2 sjemena; `python -m evaluation.compare --tag ekf15`)

| | S4 | karta točna | L1 RMSE | L1 max | L2 RMSE | L3 | NEES |
|---|---:|---:|---:|---:|---:|---:|---:|
| A realistic | 14/14 | 12/14 | 326 mm | 847 mm | 4,71° | 158 mm | 0,75 |
| C realistic | 14/14 | 12/14 | 160 mm | 408 mm | 2,41° | 76 mm | 1,66 |
| **B realistic** | 14/14 | **14/14** | **3,3 mm** | **14 mm** | **0,42°** | **7 mm** | 0,73 |
| B ideal | 14/14 | 14/14 | 2,8 mm | 12 mm | 0,41° | 6 mm | 0,48 |

M5 (14 petlji): karta točna, 14/14 zatvorenih petlji (A i C: netočna). Korekcija hodnikom drži kut i
bočnu poziciju, a raskrižja poziciju duž hodnika, pa pogreška ne raste s prijeđenim putem.

## Dinamičke prepreke i gating (scenarij ALL, M2, M3, M4, M5 × 2 sjemena)

| | S4 | karta točna | L1 RMSE | L1 max | odbačeno blizu prepreka |
|---|---:|---:|---:|---:|---:|
| A | 6/8 | 4/8 | 500 mm | 1277 mm | – |
| B s gatingom | **8/8** | **8/8** | 4,0 mm | **19 mm** | 16,9 po vožnji |
| B bez gatinga (`--no-gate`) | 8/8 | 8/8 | 4,4 mm | 41 mm | 0 |

Gating prepolovi najveću pogrešku pozicije: mjerenja bočnog senzora uz predmet (DYN3) i pri prolasku
prepreka odbacuju se. A na M4 ne nađe cilj (krivo mapiranje), B da.

Izmjena smještaja DYN4 (korak 14b): vrata moraju biti barem 0,91 m od raskrižja na drugom kraju brida
(inače bočni senzor u tom raskrižju vidi vrata kao zid, raskrižje „nestane” i karta postane
nekonzistentna – nađeno u M5 s B). Zato DYN4 traži brid od ≥ 3 koraka mreže. DYN3 ide na prvi dugi brid.

## Alati

* `--no-gate` (run, batch `--extra --no-gate`): ablacija bez gatinga.
* `python -m evaluation.compare --tag <tag> [--keys ...] [--csv]`: tablica konfiguracija iz zapisa.
* Batch CSV sada ima popis konfiguracija u imenu (ranije se `ekf15.csv` prepisivao).
