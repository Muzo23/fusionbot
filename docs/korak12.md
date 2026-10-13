# Korak 12 – Generator scene labirinta (sim/maze_scene.py)

Iz opisa labirinta (src/mazes) generator u otvorenoj CoppeliaSim sceni gradi zidove za T1, T2 ili T3
i postavlja robota na START. Iz istog opisa offline simulator i evaluacija računaju geometriju, pa su
zidovi u simulatoru, u offline modelu i u ocjeni karte isti.

    python -m sim.maze_scene T2        (iz src/, simulacija zaustavljena)

Koraci generatora:
1. briše SAMO objekte s prefiksom FB_MAZE_ (stari zidovi i oznake); pod, robot i njegove skripte
   ostaju;
2. za svaki rubni segment slobodnog prostora gradi statični, respondable kvadar debljine 0.05 m i
   visine 0.30 m, s vanjske strane slobodnog prostora, pa je slobodna širina hodnika točno 0.728712 m.
   Nazivi su FB_MAZE_wall_NNN, jer lockstep bridge pri startu simulacije upravo te objekte stavlja u
   skup za detekciju kolizije;
3. postavlja oznake čvorova (dummy, samo vizualno);
4. postavlja robota tako da je sredina osi kotača točno na STARTU, sa smjerom NORTH (provjera
   ≤ 10⁻⁵ m / 10⁻⁵ rad);
5. zapisuje oznaku labirinta u scenu (customData.fusionbotMazeId). Live provjere je čitaju i odbijaju
   pokretanje ako scena sadrži drugi labirint.

Scena se mijenja u memoriji; Ctrl+S samo ako želiš sačuvati. Jedna .ttt datoteka služi za sve
labirinte.

**Provjera istovjetnosti:** za T1, T2 i T3 generator gradi točno iste zidove kao generator iz
FusionBot2 koji je napravio scenu korištenu u koracima 5–11 (48 / 24 / 18 zidova, centri i
dimenzije jednaki do 10⁻⁹ m). Rezultati dosadašnjih live provjera zato vrijede i za novu scenu.

Offline testovi (lažni CoppeliaSim):
- brišu se samo FB_MAZE_ objekti;
- broj i svojstva zidova su ispravni;
- slobodna širina hodnika je točna;
- postavljanje robota je točno;
- ponovno generiranje zamjenjuje stari labirint.

## Live (DFS, konfiguracija A)

    python -m sim.maze_scene T2
    python -m checks.step11_live_dfs --maze T2
    python -m sim.maze_scene T3
    python -m checks.step11_live_dfs --maze T3

Ocjena je sada podijeljena:
- **navigacija:** istraženo, natrag na STARTU, bez kolizije (PASS / CHECK);
- **topološka karta:** CORRECT / INCORRECT. To je mjereni rezultat konfiguracije, a ne kriterij
  prolaza.

Offline očekivanje s live klizanjem u okretima:
- **T2:** karta ispravna, obje petlje zatvorene;
- **T3:** prsten se ne zatvori, 7 duplikata, ≈ 639 s.

## Live rezultati (6. 10. 2026, konfiguracija A)

| labirint | navigacija | karta | zatvaranja petlje (udaljenost) | drift smjera na kraju | vrijeme | najmanji razmak |
|---|---|---|---|---|---|---|
| T1 | PASS | ispravna, 23 raskrižja | – | +10.1° | 515 s | 12.6 mm |
| T2 | PASS | ispravna, 10 raskrižja / 11 bridova | 2 (0.180 m, 0.062 m) | +6.4° | 292 s | 13.9 mm |
| T3 | PASS | ispravna, 8 raskrižja | 1 (**0.236 m**, radijus 0.373 m) | +4.2° | 322 s | 11.9 mm |

Kolizija nije bilo ni u jednom pokretanju. Slike karata: figures/step11_graph_T1_A_live.png,
step12_graph_T2_A_live.png, step12_graph_T3_A_live.png.

**Napomena o slikama karte:** točke su PROCIJENJENI položaji raskrižja iz odometrije, a ne stvarni
položaj robota. Zato procjena može „izaći” iz hodnika, iako robot (GT) cijelo vrijeme vozi sredinom
hodnika. Slika figures/step11_T1_gt_vs_odometry.png prikazuje oba puta u T1: stvarni put ostaje u
hodnicima, a put prema odometriji se postupno zakreće. Na kraju je razlika 0.88 m.

## Nalaz: offline model gubitka u okretu je rekalibriran

Offline je za T3 u A bio predviđen promašaj prstena, a live ga je robot zatvorio. Analiza 68 okreta iz
tri live pokretanja (odo − GT po okretu):

| tip okreta | broj | live odo − GT | stari offline model (proporcionalan kutu) |
|---|---|---|---|
| CCW 90° | 39 | +0.46° | +0.60° |
| CW 90° | 13 | −0.66° | −0.60° |
| polukružni 180° | 16 | +0.49° | **+1.21°** |

Gubitak po okretu je STALAN, ne ovisi o kutu; to je već pokazao korak 7. Stari model ga je skalirao s
kutom i tako udvostručio pogrešku na svakom polukružnom okretu (T3 ih ima četiri). Novi offline model
(tests/plant.py, turn_deficit) daje stalan gubitak po okretu: CCW 0.47°, CW 0.66°.

Usporedba novog modela s live podacima:

| | drift T1 | drift T2 | drift T3 | zatvaranje T3 |
|---|---|---|---|---|
| live | +10.1° | +6.4° | +4.2° | 0.236 m |
| stari offline model | +15.6° | +10.8° | +18.1° | promašeno |
| **novi offline model** | +7.0° | +5.7° | +4.5° | 0.180 m |

**Rezerva za zatvaranje petlje u T3 u konfiguraciji A** (offline, gubitak po okretu skaliran):

| gubitak | 1× (live) | 1.5× | 2× | 3× |
|---|---|---|---|---|
| udaljenost pri zatvaranju | 0.18 m | 0.27 m | 0.36 m | promašeno |

Nominalna konfiguracija A zatvara prsten s polovicom rezerve. Pri 3× (npr. sklizak pod) petlja se
promaši, karta se udvostruči, a kad drift smjera prijeđe 45°, memorija više ne odgovara svijetu.
Ovo je izravni ulaz za smetnje u koraku 19.

**Zaštita konzistentnosti u DFS-u (nova):** ako izabrani manevar fizički nije dostupan na trenutnom
raskrižju (memorija ne odgovara svijetu), istraživač kontrolirano staje: FINISH s razlogom
„inconsistent”, umjesto SAFETY_ABORT. To je mjerljiv kvar procjene poze, a ne sigurnosni događaj.

## 12b – LiDAR na robotu (sim/lidar_setup.py)

Model *Hokuyo URG 04LX UG01_Fast* (2 perspektivna vizijska senzora × 256 zraka × 120° = 512 zraka, 240°),
postavljen na plavi prsten gornje ploče i postavljen skriptom:

    python -m sim.lidar_setup --keep-position            (probni prolaz)
    python -m sim.lidar_setup --apply --keep-position    (zatim Ctrl+S)

- roditelj `/FusionBot`, sredina vidnog polja = smjer robota, svi dijelovi statični i ne-respondabilni
  (masa i dinamika robota ostaju kao u koracima 1–11);
- glavna skripta modela zamijenjena skriptom `FusionBotLidarScan` (cijeli robot isključen iz renderiranja,
  domet 5 m, svaki korak simulacije objavljuje `signal.fusionbotLidarXYZD` i `signal.fusionbotLidarScanCount`);
  filter skripte ispod `sensor1`/`sensor2` ostaju (pretvaraju dubinsku sliku u udaljenosti);
- montaža (`results/lidar_mount.json`): 0.1786 m ispred osi kotača, 0.0020 m lijevo,
  ravnina skeniranja 0.2128 m iznad poda (vrh zidova 0.30 m), zakret 0.000°.

## 12c – statička provjera LiDAR-a (checks/step12c_lidar_static.py)

Robot stoji na STARTU; sken se uspoređuje sa zrakama bačenim kroz poznate zidove (WallMap) iz stvarne poze skena.
Model daje **684 zrake po 0.351°** preko 240° (kao stvarni URG-04LX), novi sken svaki korak simulacije.

Prvo mjerenje (7. 10., T3, START): nema vlastitih pogodaka, nedostajućih ni lažnih povrata, ravnina ravna,
skenovi identični. Udaljenosti su **sustavno preduge** i pogreška raste s udaljenošću
(+1.4 mm na 0.4 m, +2.6 mm na 0.5–1 m, +10 mm na 1.7 m; okomito na zid 1–2 mm na blizinu).
Zrake koje zid zahvaćaju pod malim kutom (grazing) tu pogrešku pojačavaju 1/sin(kut upada),
pa se statistika duž zrake računa samo za kut upada ≥ 30°. Posljedica: poravnanje skena sa zidovima
daje montažu 9.4 mm iza stvarne – to nije pogreška montaže nego pogreška udaljenosti senzora
(prednji zidovi su na 1.7 m). S modelom e(r) = a r + b r² procijenjenim iz tog skena montaža izlazi 0.25 mm.

## 12d – kalibracija udaljenosti (checks/step12d_lidar_calibration.py)

Robot se (simulacija zaustavljena) postavi na 10 poza – središta čvorova i osi hodnika – tako da je zid
ravno ispred na udaljenostima 0.19–4.9 m (T3), na svakoj se uzme sken. Iz svih zraka koje nisu grazing
i nisu na rubu zida procijeni se e(r) = a r + b r² (najmanji kvadrati kroz ishodište) i spremi u
`results/lidar_calibration.json`. Provjera: medijan pogreške po razredima udaljenosti nakon korekcije
< 2 mm, montaža na svakoj pozi < 2 mm / 0.2°. GT se koristi samo za kalibraciju.

    python -m checks.step12d_lidar_calibration    (iz src/, scena T3, simulacija zaustavljena)
    python -m checks.step12c_lidar_static         (zatim ponovno: kriteriji na kalibriranim udaljenostima)

Kriteriji 12c nakon kalibracije: medijan |e| < 2 mm, p95 < 8 mm (zrake ≥ 30°), p95 okomito na zid < 6 mm,
montaža < 2 mm / 0.2° – sve daleko unutar specifikacije stvarnog senzora (±30 mm do 1 m, ±3 % iznad).

Prva živa kalibracija (7. 10., T3, 10 poza, LiDAR iznad osi kotača): pogreška prije korekcije
+1.4 mm (0–0.5 m), +3.1 (0.5–1), +9.9 (1–2), +10.3 (2–3), +17.6 mm (3–5 m), dakle ≈ +0.4 % udaljenosti.
Obični najmanji kvadrati dali su a = 5.7 mm/m, b = −0.23 mm/m², ali ih je povukao razred 1–2 m:
zrake uz kutove zidova (mixed pixels – model preuzorkuje piksele dubine 0.2–0.5° na zrake 0.351°)
nose pogreške od nekoliko cm. Zato: rubne zrake se isključuju u prozoru ±3 zrake (≈1°), model se
procjenjuje robusno (MAD, 3 ponovna prilagođavanja), a poravnanje skena sužava prag na 3× rms.
Statička provjera na STARTU s tim modelom već je prošla (montaža −0.8 mm / +0.5 mm / 0.05°).

Druga kalibracija (robusna, isti položaj): a = 3.99 mm/m, b = 0.08 mm/m² (≈ +0.4 % udaljenosti),
477 od 5700 zraka odbačeno kao outlieri, rms 3.12 → 1.40 mm, medijan po razredima nakon korekcije ≤ 0.81 mm.
Poravnanje skena sa zidovima nakon korekcije daje na svih 10 poza isti mali pomak
(naprijed −1.7 ± 0.9 mm, lijevo +0.4 ± 0.1 mm, zakret +0.10 ± 0.05°): to je **ekstrinzična kalibracija**
– efektivno ishodište skena u odnosu na `lidar_mount.json`; sprema se u `lidar_calibration.json`
i dodaje montaži. Kriteriji: ekstrinzična korekcija < 5 mm / 0.5°, svaka poza unutar 2 mm / 0.2° od nje.

### Konačno stanje LiDAR-a (7. 10.)

- položaj: os kućišta `lidar_phy` (`sim.lidar_setup --onto lidar_phy --height 0.22`):
  0.1815 m ispred osi kotača, 0.0014 m lijevo, ravnina skeniranja 0.2202 m iznad poda;
- model udaljenosti: a = 4.252 mm/m, b = 0.015 mm/m² (≈ +0.43 % udaljenosti), 5083 zrake, do 4.87 m,
  528 outliera (mixed pixels) odbačeno; rms 3.14 → 1.42 mm; medijan po razredima nakon korekcije ≤ 0.77 mm;
- ekstrinzična korekcija: naprijed −2.03 ± 1.13 mm, lijevo +0.40 ± 0.20 mm, zakret +0.119 ± 0.080°;
  najveće odstupanje poze od nje 2.61 mm / 0.46 mm / 0.157° → **12d PASS**;
- statička provjera na STARTU nakon kalibracije: bias +0.07 mm, medijan |e| 0.87 mm, p95 4.56 mm,
  montaža −0.51 mm / +0.05 mm / −0.061° → **12c PASS**;
- slika: `docs/figures/step12d_lidar_range_error_live.png`, tablica poza: `tests/data/step12d_calibration_poses_live.csv`.

Sljedeće (13b): vremensko usklađivanje skena i odometrije u okretu te model šuma stvarnog
URG-04LX-UG01 (±30 mm do 1 m, ±3 % iznad, 10 Hz) u Pythonu, primijenjen na kalibrirane udaljenosti.

## 12e – IMU u kućištu IMU_phy (sim/imu_setup.py, checks/step12e_live_imu.py)

Svi senzori konfiguracija A–E fizički su na robotu: enkoderi kotača (zglobovi), 5 senzora blizine,
LiDAR u `lidar_phy` i IMU u `IMU_phy` (kućište kamere `cameraHousing_phy` ostaje prazno – kamera nije
dio nijedne konfiguracije).

`python -m sim.imu_setup --apply` stvara dummy `/FusionBot/FusionBotIMU` u središtu kućišta `IMU_phy`
(osi = osi robota) i skriptu `FusionBotImuScript`, koja svaki korak (nakon koraka fizike) iz poza
zadnja tri koraka računa **idealni** 6-osni IMU:

- žiroskop: kutna brzina u okviru tijela iz R(k−1)ᵀR(k);
- akcelerometar: specifična sila f = R(k)ᵀ(a − g) iz druge razlike položaja (u mirovanju (0, 0, +9.81)).

Objavljuje `signal.fusionbotImuGyro`, `signal.fusionbotImuAccel`, `signal.fusionbotImuCount`;
montaža u `results/imu_mount.json`. Šum i bias stvarnog MEMS IMU-a dodaju se u Pythonu (korak 13),
kao i za LiDAR. Formule su u Pythonu zrcaljene u `imu_from_poses` i testirane (mirovanje, kružnica,
ubrzanje naprijed).

Živa provjera (misija iz koraka 7: START→ROOT pa +90/−90/+180/−180°): žiroskop z prema GT brzini
zakretanja (< 1e-3 rad/s, bez kašnjenja), žiroskop x/y mali, akcelerometar u mirovanju 9.81,
u vožnji prema specifičnoj sili očekivanoj iz GT putanje osi (a_x = du/dt − ω²r, a_y = uω + ω̇r).

## 12f – dijelovi robota grupirani u mainBase_phy (sim/robot_parts.py)

Od 8. 10. dijelovi robota (kućišta, stupići, ploča …) grupirani su u jedan oblik (`mainBase_phy`), pa
`IMU_phy` i `lidar_phy` više nisu zasebni objekti. `find_part(sim, robot, "imu" | "lidar", floor_z)`
pronalazi kućište: najprije kao zasebni oblik, inače **kopira** svaki oblik robota, kopiju razgrupira
(`sim.ungroupShape`) i među komponentama traži onu poznate veličine (IMU 0.062 × 0.064 × 0.019 m,
LiDAR 0.075 × 0.075 × 0.045 m, sortirane dimenzije, tolerancija 6 mm, visina > 0.12 m); sve kopije
(i ono što su donijele) se brišu – robot se ne mijenja. Koriste ga `sim.imu_setup` i
`sim.lidar_setup --onto lidar|auto`. Testovi s lažnim simulatorom: `tests/test_step12f_robot_parts.py`.

Napomena: ako je grupiranje promijenilo masu ili tromost tijela, ponoviti provjeru okreta (korak 7).

**Važno (8. 10.):** `/FusionBot` je statična baza modela i u simulaciji se NE giba; giba se tijelo
`mainBase_phy`. LiDAR (`fastHokuyo`) i IMU (`FusionBotIMU`) zato moraju biti djeca `mainBase_phy`
(kao senzori blizine) – `lidar_setup` i `imu_setup` ih tamo stavljaju. Statičke provjere i kalibracija
LiDAR-a (12c/12d) ostaju valjane jer robot u njima stoji, a položaj LiDAR-a se ne mijenja.

### Živa provjera IMU-a (8. 10., T1, misija koraka 7)

- 2452 koraka, svaki ima uzorak IMU-a; skripta se izvršava prva u koraku;
- **žiroskop z prema GT brzini zakretanja: max |razlika| 3.5e-5 rad/s, rms 1.5e-5, bez kašnjenja** (vrh 1.23 rad/s);
- žiroskop x/y: u mirovanju ≤ 0.0007 rad/s; u vožnji tijelo se ljulja na kontaktima (rms 0.013, max 0.052 rad/s)
  – stvarno gibanje, ravninski EKF koristi samo g_z;
- akcelerometar u mirovanju: z = 9.8101 m/s², |xy| srednje 0.012 m/s²;
- akcelerometar u vožnji prema GT (50 ms): rms 0.030 (x) i 0.049 m/s² (y); bočno vibracije ±0.2 m/s² u okretima
  (kontakti MuJoCo) – još jedan razlog da akcelerometar ne ulazi u procjenu poze.
- Prvi kriteriji „mirovanja“ bili su krivi (nulta naredba ≠ mirovanje, robot još koči; ljuljanje u vožnji
  nije pogreška senzora) – ispravljeno; slika `docs/figures/step12e_imu_live.png`.
