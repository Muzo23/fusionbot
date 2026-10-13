# Korak 15 – jezgra EKF-a i konfiguracija C (žiroskop + enkoderi)

**Zahtjev zadatka:** „implementacija naprednih algoritama za fuziju podataka iz više izvora kako bi se
osigurala precizna procjena stanja robota u uvjetima nesigurnosti”.

## Filtar (`localization/ekf.py`)

Jedan prošireni Kalmanov filtar za sve konfiguracije; konfiguracija bira ulaze i korekcije.

* Stanje x = [x, y, θ, b_g] (b_g = pomak nule žiroskopa).
* **Predikcija** svakih 10 ms: put ds iz enkodera; zakret iz enkodera (A, B, D) ili iz žiroskopa
  (ω − b_g)·dt (C, E); integracija po srednjem kutu (ista kao odometrija koraka 4); P = F P Fᵀ + G Q Gᵀ.
* **Šum procesa** raste s prijeđenim putem i zakretom (ne s brojem koraka):
  put 1 cm/√m; enkoderski kut 1° po okretu od 90° (izmjereni gubitak u okretu 0,86° / 0,97°) i
  0,3°/√m ravno; žiroskop šum 0,05 °/s i 0,3° po 90° (ostatak faktora skale 0,3 %); bias kao slučajni hod
  0,002 °/s/√s. Robot ne zna u kojem profilu šuma vozi, pa su vrijednosti iste za ideal i realistic.
* **ZUPT** (C, E): kad enkoderi pokazuju mirovanje (isti test kao FSM, 13b-8), žiroskop mjeri samo svoj
  bias → korekcija b_g. Početni bias daje startna kalibracija (0,5 s mirovanja), s nesigurnošću σ/√N.
* **Gating** svake korekcije: NIS = νᵀS⁻¹ν uspoređuje se s χ² (99 %); veće → mjerenje odbačeno (outlier).
  Isti mehanizam koristit će PROX-LOC i LiDAR (prepreke iz 14b). Kovarijanca se ažurira Josephovim oblikom.
* Konfiguracija A sada je ista jezgra bez korekcija: srednja vrijednost je točno odometrija koraka 4
  (test), a filtar dodaje kovarijancu, pa i A ima metriku konzistentnosti.

Literatura: Thrun, Burgard, Fox – *Probabilistic Robotics* (2005), pogl. 3 i 5; Bar-Shalom, Li,
Kirubarajan – *Estimation with Applications to Tracking and Navigation* (2001), NEES/NIS i gating.

## Nova metrika L5 (konzistentnost)

NEES = eᵀP⁻¹e za [x, y, θ] (GT samo za evaluaciju). Za konzistentan filtar srednja vrijednost je 3, a
95 % uzoraka ispod 7,815. Manje od 3 = filtar je previše oprezan; više = previše samouvjeren.
Kovarijanca se zapisuje u svaki tick (`P_xx … P_tt`, `est_bg`).

## Rezultati offline (batch `ekf15`: T1–T3, M1, M2, M3, M5 × 2 sjemena)

| | L1 RMSE | L2 RMSE | L3 raskrižja | L4 drift | T karta točna | NEES (srednja) |
|---|---:|---:|---:|---:|---:|---:|
| A ideal | 326 mm | 4,71° | 158 mm | 0,60 % | 12/14 | 0,75 |
| A realistic | 325 mm | 4,71° | 158 mm | 0,60 % | 12/14 | 0,75 |
| C ideal | **24 mm** | **0,36°** | 13 mm | 0,04 % | 14/14 | 0,05 |
| C realistic | **160 mm** | **2,41°** | 76 mm | 0,25 % | 12/14 | 1,66 (max 7,9) |

* S4 56/56. Žiroskop smanjuje pogrešku pozicije 2× (realistic) do 13× (ideal); kut 2× do 13×.
* C realistic: ostatak dolazi od faktora skale žiroskopa (izvučen po sjemenu, σ 0,3 %) – bez apsolutne
  korekcije (zidovi, LiDAR) kut i dalje sporo drifta. M5 (14 petlji) i dalje ima netočnu topološku kartu
  i u C → potrebne su korekcije B/D/E.
* A je u oba profila jednak: enkoderi su isti u oba profila (kvantizacija), a šum senzora blizine
  utječe samo na vođenje.
* NEES: A je oprezan (0,75 < 3), C realistic u jednoj vožnji previše samouvjeren (7,9). Podešavanje Q
  prema NEES-u preko Monte Carla je korak 15b (kandidat za evolucijsko podešavanje).
* ZUPT: npr. T1 realistic 2617 prihvaćenih, 1003 odbačenih korekcija (robot se još njiše nakon okreta,
  gate ih odbacuje).
