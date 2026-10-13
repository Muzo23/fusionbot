# FusionBot – sažetak za nastavak u novom chatu (12. 10. 2026., nakon koraka 15b)

## 1. Rad i pravila suradnje
- Diplomski rad: **„Povećanje pouzdanosti autonomne navigacije primjenom metoda fuzije podataka u kompleksnim okruženjima”**, Leo Muzica, TF Rijeka, mentor doc. dr. sc. Nikola Anđelić, predmet Evolucijska robotika. Rok predaje: zadnji tjedan listopada 2026.
- Tekst zadatka: projektiranje i validacija lokalizacije i navigacije u **dinamičkom i kompleksnom** simulacijskom okruženju; **fuzija podataka iz više izvora**; precizna procjena stanja u nesigurnosti; **stohastičke smetnje** (sustavni šum, nelinearni vanjski utjecaji na kretanje); samostalno rješavanje prostornih zadataka uz **modeliranje nepoznatog okruženja** ili slijeđenje putanje.
- Pravila:
  1. Uz svaki korak objasniti što se postiže i zašto, je li provjera infrastrukture ili završni test, i što radi svaka naredba.
  2. Uvijek pisati `cd`: testovi se pokreću iz `fusionbot`, a run, checks i evaluation iz `fusionbot\src`.
  3. **Uvijek provjeriti usklađenost sa zadatkom i Uputama** i navesti koji zahtjev korak pokriva.
  4. Odgovori na hrvatskom.
- Pisanje rada: korisnik piše u svom LaTeX-u (`main.tex`); pisanje je zasad pauzirano. Prijedlog je poglavlje 3 pisati paralelno.
- Korisnikovo računalo: `C:\Users\Agenor\Desktop\Diplomski Rad\fusionbot`, okruženje „FusionBot”, CoppeliaSim 4.10 + MuJoCo.

## 2. Sustav (repozitorij `fusionbot`, korak 17 = commit 1949457; korak 15b u paketu `fusionbot_15b.zip`, za commit)
- **Robot:** diferencijalni pogon; r = 0,052975 m, B = 0,2901 m (izmjereno), B_eff = 0,3352 m (za okrete); 5 senzora blizine, IMU MPU-6050, LiDAR Hokuyo URG-04LX (10 Hz, 684 zrake). Brzina 0,25 m/s, dt 10 ms.
- **Bridge:** lockstep ZMQ, protokol 2.2 (IMU svaki korak, LiDAR svaki 10., položaji dinamičkih prepreka u paketu naredbe). Nakon promjene `python -m sim.install_bridge` + Ctrl+S.
- **Navigacija (ista u svim konfiguracijama):**
  - FSM (FOLLOW / APPROACH / POSITION / TURN / EXIT / BLOCKED);
  - vođenje po hodniku s prediktivnim filtrom mjerenja (13b-8);
  - DFS istraživanje → povratak na START → Dijkstra do cilja (misija);
  - topološka karta (MazeMemory) i karta zauzetosti (2 cm, log-odds, K2).
- **Labirinti:** T1–T3 (vlastiti), M1–M7 (micromouse, github micromouseonline/mazefiles 548dcdeb), mreža koraka 1,085268 m, hodnik 0,728712 m.
- **Profili šuma:** ideal i realistic.
  - Žiroskop MPU-6050: 0,05 °/s, bias ±20 °/s, skala σ 0,3 %.
  - LiDAR: ±30 mm / ±3 % (2σ).
  - Senzori blizine: 2 mm + 1 % (pretpostavka).
  - Enkoderi: 1024 CPR.
- **Konfiguracije (razlikuje se samo procjena poze):**

| | izvori | korak |
|---|---|---|
| A | enkoderi (EKF bez korekcija = odometrija + kovarijanca) | 15 |
| B | + PROX-LOC: zidovi hodnika (y, α) i raskrižja prema mreži labirinta | 16 |
| C | + žiroskop za kut, ZUPT za bias | 15 |
| D | + LiDAR, poravnanje skena s vlastitom kartom (likelihood field + Gauss-Newton) | 17 |
| E | sve | 17 |

- **EKF** (`localization/ekf.py`): stanje [x, y, θ, b_g]; šum procesa raste s putem i kutom; χ² gate 99 % na svakoj korekciji; Josephov oblik; `--no-gate` za ablaciju.
- **15b – podešeno diferencijalnom evolucijom** (replay, NEES + L1): put 2,0 mm/√m (bilo 10), okret 0,71°/90° (1°), ravno 0,056°/√m (0,3°), žiroskop 0,26°/90° (0,3°), GYRO_BIAS_RW nepromijenjen (tablica podataka), COV_INFLATE 100 (1000). Alati: `experiment/replay.py`, `experiment/tune.py`; dokumentacija `docs/korak15b.md`, podaci `docs/korak15b_data/`.
- **Pretpostavka za B (navesti u radu):** poznata pravila gradnje labirinta, kao u micromouse natjecanjima (osi, mreža, startna ćelija). D tu pretpostavku ne koristi.
- **Dinamičke prepreke (14b, `--dyn DYN1..4|ALL`):** pojavljuju se u vožnji do cilja, na isplaniranoj ruti.
  - DYN1: predmet prelazi hodnik.
  - DYN2: vrata zatvorena 8 s.
  - DYN3: predmet uz zid.
  - DYN4: trajna vrata, na bridu ≥ 3 koraka mreže; robot nakon 15 s okreće i planira obilazak.
  - Otkrivanje: FRONT senzor uspoređuje očitanje s odometrijskom duljinom brida.
- **Metrike** (`evaluation.metrics`):
  - S1–S4 (uspjeh misije), L1–L5 (L5 = NEES), T (topološka karta), K1/K2 (metrička karta), DYN, E, P1–P8.
  - Tablica konfiguracija: `python -m evaluation.compare --tag <tag>`.

## 3. Rezultati
**Uživo (CoppeliaSim, T1 realistic s1):**

| | L1 RMSE | L1 max | L2 | K1 | K2 | NEES | karta |
|---|---:|---:|---:|---:|---:|---:|---|
| A (live14) | 421 mm | 1083 mm | 7,05° | 248 mm | 55,7 % | – | točna |
| C (live15) | 176 mm | 560 mm | 2,86° | 128 mm | 70,9 % | 2,76 | točna |
| **B (live16)** | **5 mm** | **15 mm** | 0,74° | **19 mm** | **86,2 %** | 2,44 | točna |

- Filtar vođenja uživo: P6 0,71 rad/s² i 2,5 promjene smjera po metru (prije 2,83 i 64).
- Prepreke uživo (M3 A ideal, live14b): S4 uspjeh, 3 zaustavljanja, 2 nastavka, 1 okret, 0 lažnih zaustavljanja, karta točna; isto kao offline.
- Faktor stvarnog vremena uživo: 1,01 na T1 A, ali 0,57–0,67 u zadnjim vožnjama (M3 i T1 B/C). Treba provjeriti uzrok (`checks.step13b_speed`).

**Offline:**
- `ekf15`, realistic, 7 labirinata × 2 sjemena:
  - L1: A 326 mm, C 160 mm, B 3,3 mm;
  - točna karta: A 12/14, C 12/14, B 14/14.
- Dinamika `ALL_dyn16`, M2–M5 × 2:
  - A: S4 6/8, točna karta 4/8;
  - B: S4 8/8, točna karta 8/8;
  - najveća pogreška pozicije u B: 19 mm s gatingom, 41 mm bez.
- T1 s LiDAR-om: L1 A 340 mm, D 63 mm, E 3 mm; K2 A 60 %, D 84 %, E 89 %.
- **15b (replay 21 vožnje, validacija T2/M1/M3), poslije podešavanja:**
  - L1 / ANEES (validacija): A 183 mm / 1,47 (bilo 0,60); B 3,1 mm / 1,18 (0,77); C 86 mm / 1,57 (1,09); D 48 mm / 234 (60 mm / 40); E 3,2 mm / 2,04 (0,81).
  - D se ne može učiniti konzistentnim podešavanjem (karta iz iste procjene → strukturna nekonzistentnost); COV_INFLATE odabran prema točnosti. Isprobani CI (Covariance Intersection): ANEES 3, ali L1 0,5–1,4 m → odbačen.
  - Zatvorena petlja `tuned15b` (B, D, E × 7 × s1): 7/7 uspjeh i točna karta; L1 B 3,0 / D 61 / E 3,1 mm. `ALL_dyn15b`: A 6/8, B 8/8 (najveća pogreška 17 mm), 0 lažnih zaustavljanja.
  - 212 offline testova prolazi.
- Batch `lid17` (A–E × 7 labirinata × 2 sjemena, s LiDAR-om) je u tijeku na serveru (stari parametri – za usporedbu prije/poslije 15b ponoviti s tagom `lid15b`).
  - Ako rezultati nisu dostupni, ponoviti iz `src`: `python -m experiment.batch --config A B C D E --noise realistic --mazes T1 T2 T3 M1 M2 M3 M5 --seeds 1-2 --tag lid17 --extra --offline-lidar`.
  - Usporedbu složiti s `python -m evaluation.compare --tag lid17`.

## 4. Otvoreno / čeka
- Korisnik: vožnje uživo D i E (T1 realistic, tag live17); `checks.robot_geometry` v2; vrsta kotačića.
- **Pitati mentora:** je li dovoljan evolucijski dio iz 15b (diferencijalna evolucija nad parametrima šuma EKF-a, NEES + L1, Monte Carlo replay – već napravljeno) ili želi više (npr. usporedba DE i pretraživanja mreže, ili evolucija parametara vođenja); prihvaća li pretpostavku o mreži labirinta za B.
- **Potvrditi (15b):** COV_INFLATE za D odabran prema točnosti (100), iako bi unaprijed zadana funkcija cilja s NEES članom birala 1000 – D je nekonzistentan za svaku vrijednost (ANEES ≥ 140).
- Uživo provjeriti NEES s novim Q (live17 D/E i ponoviti T1 A/C): offline pogon nema proklizavanja, uživo Q možda treba povećati.

## 5. Plan do kraja
1. ~~15b – podešavanje šuma EKF-a i COV_INFLATE (DE na replayu)~~ – gotovo 12. 10.
2. 19 – smetnje D1–D7 (sklizak pod, polumjer i razmak kotača, proklizavanje, skok biasa, degradacija LiDAR-a, kvar bočnog senzora) × A–E × realistic × T3 i M4 × 3 sjemena – 1 dan
3. 20 – završni eksperimenti: uživo (A–E na odabranim labirintima, s preprekama i bez) + offline Monte Carlo – 2 dana
4. analiza, tablice i slike za rad – 1 dan
5. pisanje (poglavlja 3 i 4 iz `docs/korak*.md`), pregled, predaja

## 6. Datoteke za pisanje
- `docs/korak13.md`, `korak14.md` (karta + 14b dinamika), `korak15.md` (EKF + C), `korak15b.md` (podešavanje šuma, DE), `korak16.md` (B), `korak17.md` (D, E).
- Projekt na claude.ai: `claude/odluke-i-stanje.md`.
