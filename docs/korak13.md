# Korak 13 – konfiguracije, profili šuma i jedan runner

## 13a – tko u čemu sudjeluje (`core/configurations.py`)

Sve konfiguracije voze **isti robot istom navigacijom** (FSM, vođenje po hodniku senzorima blizine,
DFS) i crtaju **istu metričku kartu LiDAR-om**. Razlikuju se samo po tome što ulazi u **procjenu poze**:

| role | A | B | C | D | E |
|---|---|---|---|---|---|
| distance (prediction) | encoders | encoders | encoders | encoders | encoders |
| heading (prediction) | encoders | encoders | gyro | encoders | gyro |
| gyro bias (ZUPT) | - | - | yes | - | yes |
| PROX-LOC update | - | yes | - | - | yes |
| LiDAR update | - | - | - | yes | yes |
| corridor guidance (prox) | yes | yes | yes | yes | yes |
| metric map (LiDAR) | yes | yes | yes | yes | yes |
| estimator | odometry | ekf | ekf | ekf | ekf |

- **enkoderi**: put Δs u svim konfiguracijama; kut Δθ = (Δs_D − Δs_L)/B samo u A, B, D;
- **žiroskop z**: kut u predikciji (C, E); bias se procjenjuje u mirovanju (ZUPT) i na startu
  (statička kalibracija 0.5 s);
- **senzori blizine**: vođenje po hodniku u svim konfiguracijama; ažuriranje poze (PROX-LOC) u B i E;
- **LiDAR**: metrička karta u svim konfiguracijama (crta se po procijenjenoj pozi, pa se razlike u pozi
  vide na karti); ažuriranje poze poravnanjem skena s kartom u D i E;
- **akcelerometar**: samo se zapisuje (detekcija mirovanja/proklizavanja – budući rad);
- **GT**: samo evaluacija i kalibracija.

## Profili šuma (`sensors/noise_models.py`)

Svaka konfiguracija vozi se **bez šuma (ideal)** i **sa šumom (realistic)**. Simulator daje idealna
mjerenja; realističan profil u Pythonu dodaje pogreške stvarnih dijelova iz tehničke specifikacije.
Sva slučajnost dolazi iz jednog sjemena po vožnji (odvojeni tokovi po senzoru), pa je vožnja
ponovljiva, a Monte Carlo mijenja samo sjeme.

| senzor | ideal | realistic |
|---|---|---|
| enkoderi | kvantizacija 1024 CPR | isto (kvantizacija je vlastita pogreška enkodera) |
| žiroskop z | stvarna brzina zakretanja | MPU-6050 (±250 °/s): šum 0.05 °/s rms, LSB 1/131 °/s, početni pomak nule ±20 °/s (uklanja ga startna kalibracija) – iz tehničke specifikacije; hod biasa 0.002 °/s/√s i preostala pogreška faktora skale 0.3 % – **pretpostavke** |
| LiDAR | kalibrirane udaljenosti simulatora (12d) | URG-04LX-UG01: ±30 mm (do 1 m), ±3 % (1–4 m) kao 2σ, rezolucija 1 mm |
| senzori blizine | točna udaljenost | σ = 2 mm + 1 % udaljenosti, rezolucija 1 mm – **pretpostavka** dok se ne odabere stvarni dio |

Realistični profil djeluje na fizički senzor, dakle i na vođenje po hodniku u svim konfiguracijama;
robusnost FSM-a na šum provjerava se offline (MazeSim) prije živih vožnji.

## Plan završnih testova

| skup | konfiguracije | profil | labirinti | ponavljanja |
|---|---|---|---|---|
| nominalno | A–E | ideal | T1, T2, T3 | 1 (deterministički) |
| sa šumom | A–E | realistic | T1, T2, T3 | 3 sjemena uživo + Monte Carlo offline (korak 20) |
| smetnje (korak 19) | A–E | realistic | T3 | gubitak u okretu ×k, polumjer kotača, skok biasa |

Metrike: pogreška poze (pozicija, kut) kroz vrijeme, pogreška položaja čvorova, ispravnost topološke
karte, rezerva zatvaranja petlje, kvaliteta metričke karte (prema GT karti), vrijeme istraživanja.

## 13b-1 – bridge protokol 2.1 (`sim/protocol.py`, `sim/bridge_script.py`)

Stari paket (52 vrijednosti) ostaje isti, a na njega se dodaje proširenje:

| indeks | sadržaj |
|---|---|
| [52] | verzija proširenja (1) |
| [53] | brojač IMU uzoraka (−1 ako u sceni nema IMU-a) |
| [54..56] | žiroskop x, y, z [rad/s] – idealne vrijednosti, okvir IMU-a |
| [57..59] | akcelerometar x, y, z (specifična sila) [m/s²] |
| [60] | brojač LiDAR skenova (−1 ako u sceni nema LiDAR-a) |
| [61] | n = broj zraka priloženih **ovom** paketu |
| [62..] | n × (x, y, z, d) u okviru Hokuyo senzora |

* IMU stiže u **svakom** koraku (100 Hz). Sken se prilaže samo svakom 10. paketu (10 Hz, kao stvarni
  URG-04LX), a u ostalim paketima je n = 0.
* Šum se **ne** dodaje u simulatoru, nego u Pythonu (`sensors/noise_models.py`). Zato isti zapis
  vožnje daje i idealnu i zašumljenu verziju.
* Skripte IMU-a i LiDAR-a izvršavaju se prve u koraku (exec priority). Bridge tako u istom koraku
  šalje svježe vrijednosti, a razlika (brojač LiDAR-a − brojač IMU-a) ostaje stalna.
* Paket bez proširenja (bridge 2.0) i dalje se prihvaća. U tom slučaju polja proširenja su prazna.
* Provjera uživo: `python -m checks.step13b_live_sensors` (step-7 misija; uključuje i ponovljenu
  tablicu okreta nakon grupiranja dijelova u mainBase_phy, točka 13b-4).

Rezultat uživo (8. 10. 2026.): **PASS**. Ovo je provjera infrastrukture, a ne završni test.

- IMU stiže u 2452 od 2452 paketa, brojač raste za točno 1 po paketu.
- Žiroskop z u paketu odstupa od GT najviše 3.45e-5 rad/s. To je isto kao u koraku 12e, pa bridge ne
  unosi nikakvu pogrešku.
- LiDAR je u 245 skenova po 684 zrake, svaki 10. paket.
- Brojač skena, brojač IMU-a i redni broj paketa su jednaki. Svi podaci u paketu su dakle iz istog
  koraka simulacije.
- Najlošiji sken ima 92 % pogodaka zida.

**13b-4 – okreti nakon grupiranja dijelova u mainBase_phy:** ponašanje se promijenilo.

| zadano | odo − GT prije (5. 10.) | GT/odo prije | odo − GT sada (8. 10.) | GT/odo sada |
|---:|---:|---:|---:|---:|
| +90° | +0.36° | 0.996 | −0.49° | 1.0055 |
| −90° | −0.96° | 0.989 | +0.43° | 1.0048 |
| +180° | +0.59° | 0.997 | −1.20° | 1.0067 |
| −180° | −1.10° | 0.994 | +1.42° | 1.0079 |

Prije grupiranja robot se okretao **manje** nego što su pokazivali enkoderi, uz stalnu pogrešku od
≈ 0.6° po okretu. Sada se okreće **više**, za oko 0.5–0.8 %, a pogreška raste s kutom. To izgleda kao
pogreška mjerila (efektivni razmak kotača), a ne kao prijelazna pojava. Grupiranjem su se promijenili
masa, tromost i točke dodira.

Posljedica: model gubitka u okretu u offline simulatoru i svi offline rezultati konfiguracije A
(13c) vrijede za stari robot. Treba ponoviti korak 7b i ponovno kalibrirati model.

**Ponovljeni 7b (8. 10. 2026.), 12 okreta:**

| okreti | srednja \|odo−GT\| | GT/odo |
|---:|---:|---:|
| 90° (8) | 0.42° | 1.002–1.006 |
| 180° (4) | 1.49° | 1.005–1.010 |

Pogreška sada raste s kutom, ali ne razmjerno: okret od 180° nema dvostruku, nego 3.5 puta veću
pogrešku od okreta od 90°. Okreti su zato opisani modelom s dva dijela:

    |GT| = k · |odo| − gubitak(smjer)

Rezultat prilagodbe metodom najmanjih kvadrata:

- k = 1.0119 je pogreška **mjerila**: robot se okreće 1.19 % više nego što pokazuju enkoderi.
- **Stalni gubitak** po okretu: CCW 0.56°, CW 0.74°.
- Najveće odstupanje modela od mjerenja je 0.43°.

Stalni gubitak gotovo je jednak gubitku starog robota (0.47° / 0.66° na 68 okreta u labirintima).
Prijelazno proklizavanje pri pokretanju okreta dakle se nije promijenilo. Grupiranje je promijenilo
samo mjerilo.

**Odluka.** Isto pravilo kao u koraku 7: B_eff ispravlja samo dio pogreške razmjeran kutu.

- B_eff = 0.3392 / 1.0119 = **0.3352 m**, u `core/robot_config.py`.
- Stalni gubitak ostaje u odometriji. To je pogreška koju fuzija treba ispraviti.
- Prijedlog skripte 7b (B_eff = 0.33725 m iz okreta REPEAT) se ne prihvaća. Taj prijedlog miješa
  oba dijela pogreške: gotovo bi poništio pogrešku okreta od 90°, a okreti od 180° imali bi pogrešku
  suprotnog predznaka.

**Predviđanje za provjeru uživo** (7b s novim B_eff): odo − GT ≈ +0.56° za svaki CCW okret i ≈ −0.74°
za svaki CW okret, jednako za 90° i za 180°. Skripta 7b sada sama ispisuje parametre modela (k i
gubitak), pa u novom pokretanju k mora biti ≈ 1.000.

**Provjera uživo s B_eff = 0.3352 m (7b, 8. 10. 2026.):**

| okreti | srednja \|odo−GT\| prije (B 0.3392) | sada (B 0.3352) |
|---:|---:|---:|
| 90° (8) | 0.42° | 0.68° |
| 180° (4) | 1.49° | 0.44° |

- Novi k = 1.0026. Pogreška mjerila smanjena je s 1.19 % na 0.26 %, ispod kriterija prihvaćanja od
  0.5 %. B_eff se dalje ne iterira: svaka promjena B mijenja i naredbe kotačima, pa bi se gonio šum.
- Pogreška više ne raste s kutom. Okret od 180° sada ima čak manju pogrešku od okreta od 90°, što
  potvrđuje da je ostatak prijelazna pojava, a ne mjerilo.
- Neto smjer nakon serije A (4 × +90° i 4 × −90°) je +0.02°.
- Predviđanje je kvalitativno potvrđeno: k ≈ 1 i otprilike stalna pogreška po okretu. Brojčano je
  gubitak veći od predviđenog: CCW 0.86° (predviđeno 0.56°), CW 0.97° (predviđeno 0.74°). Razlog je
  što novi B mijenja i to kako regulator vozi kotače u okretu, pa se mijenja i proklizavanje. Model
  s dvije amplitude nije potpuno neovisan o B.

**Offline model** (`tests/maze_sim.py`), prilagođen ovom mjerenju: k = 1.00263 (`LIVE_TURN_SCALE_SLIP`)
i gubitak CCW 0.857°, CW 0.974° (`LIVE_TURN_DEFICIT`). Offline 7b daje +0.62° / −0.74° za okrete od
90° i +0.39° / −0.51° za okrete od 180°. Uživo je izmjereno +0.68° / −0.68° i +0.27° / −0.61°.

Offline misija u konfiguraciji A s tim modelom (8. 10.):

| labirint | rezultat |
|---|---|
| T1, T2, T3, M1, M2 | cilj |
| M3 | **sudar** |
| M4 | **cilj nije u karti** (karta s duplikatima) |
| M5 | cilj |

Sudar s pogrešnom kartom (M3) pokazuje da je potrebna robusnost FSM-a (13b-6). Robot ne smije
udariti u zid samo zato što mu je karta kriva, jer ga vode senzori blizine.

## 13b-2 – pokretač eksperimenata i zapisnik (`run.py`, `experiment/`)

Jedna naredba pokreće jednu vožnju za odabranu kombinaciju konfiguracije, profila šuma, labirinta i
sjemena:

    python -m run --config A --noise realistic --maze M2 --seed 3            (uživo, CoppeliaSim)
    python -m run --config A --noise realistic --maze M2 --seed 3 --offline  (offline model)

Dijelovi:

* **`experiment/core.py` – `RobotRun`.** Jedan kôd robota, neovisan o izvoru mjerenja. Redoslijed u
  svakom koraku od 10 ms je isti kao u svim provjerama uživo od koraka 5:
  1. senzori (+ šum profila);
  2. enkoderi i odometrija;
  3. procjenitelj konfiguracije;
  4. FSM;
  5. PI regulatori kotača;
  6. naredbe motorima.

  GT služi samo za zapis i evaluaciju.
* **`experiment/drivers.py`.** `live_loop` radi s bridgeom 2.1 u CoppeliaSimu. `offline_loop` radi s
  offline modelom pogona i sadrži model gubitka u okretu izmjeren uživo. U offline načinu žiroskop daje
  stvarnu kutnu brzinu tijela, a LiDAR je opcionalan (`--offline-lidar`, ray-cast).
* **`localization/estimators.py`.** Konfiguracija A koristi odometriju. B–E čekaju EKF (korak 15) i
  dotad javljaju `NotImplementedError`.
* **`experiment/recorder.py`.** Svaka vožnja dobiva mapu
  `results/runs/<labirint>_<konf>_<šum>_s<sjeme>[_offline][_<tag>]/` s ovim sadržajem:

  | datoteka | sadržaj |
  |---|---|
  | `meta.json` | konfiguracija; profil šuma s izvučenim vrijednostima (npr. početni bias žiroskopa); labirint; konstante robota; montaža i kalibracija LiDAR-a |
  | `ticks.csv.gz` | jedan red po koraku: GT, procjena, naredbe, enkoderi, žiroskop idealno/izmjereno, akcelerometar, senzori blizine idealno (`d_*`) i izmjereno (`dm_*`), vođenje, razmak od zida |
  | `events.csv` | svako raskrižje: stvarni čvor, odluka, pridruživanje, pogreška poze |
  | `scans.npz` | LiDAR na 10 Hz: kutovi zraka (jednom), idealni (kalibrirani) i izmjereni dometi (uint16, 0.1 mm), GT i procijenjena poza |
  | `map_topo.json` / `.png` | topološka karta robota preko stvarnih zidova |
  | `summary.json` | osnovni rezultati (S1–S4, T, L, E, najmanji razmak, QA LiDAR-a); 13b-3 ih proširuje na sve metrike |

* **Snima se i idealna i izmjerena vrijednost svakog senzora sa šumom.** Utjecaj šuma može se zato
  analizirati bez nove vožnje.
* **Isto sjeme daje bit po bit istu vožnju** (offline test). Monte Carlo zato mijenja samo sjeme.
* **LiDAR točke.** Skripta ih objavljuje u okviru baze Hokuyo senzora. `lidar_setup` sada u
  `lidar_mount.json` zapisuje i položaj okvira skena u tom okviru (`scan_centre_in_base`,
  `scan_rotation_from_base`). Runner tako računa smjer i domet zrake bez matrice senzora, na isti način
  kao kalibracija u 12d.
* **QA LiDAR-a u svakoj vožnji.** Za 24 zrake po skenu kalibrirani domet uspoređuje se sa stvarnim
  zidovima iz GT poze. To je stalna provjera montaže, kutova i kalibracije.

Offline provjera (8. 10. 2026.): A, ideal, T2 → misija uspješna, karta točna, 326 s simulirano za 11 s.

**Provjera uživo (8. 10. 2026.), `python -m run --config A --noise ideal --maze T2`: PASS.**

* Misija je uspješna (S1–S4) i karta je točna: 10 raskrižja, 11 od 11 bridova, 2 zatvorene petlje.
* Najmanji razmak tijela od zida je 17 mm.
* Simulirano je 346 s, a stvarno je trajalo 545 s.
* Pogreška procjene u raskrižjima: srednje 136 mm, najviše 399 mm (u START-u, nakon istraživanja).
  Na cilju je pogreška 298 mm i +4.97°.
* Usporedba s offline modelom:

  | | uživo | offline |
  |---|---:|---:|
  | istraživanje | 292 s | 273 s |
  | vožnja do cilja | 53 s | 52 s |
  | pogreška na kraju | 298 mm | 409 mm |

* Snimljeno je 3450 skenova. QA LiDAR-a (kalibrirani domet prema stvarnim zidovima): medijan 1.1 mm,
  p90 6.5 mm.
* Projekcija snimljenih skenova po GT pozi: 99.3 % točaka leži unutar 2 cm od stvarnog zida (medijan
  1.1 mm). Kutovi zraka, montaža i okvir skena su dakle ispravni.
* Projekcija istih skenova po procijenjenoj pozi (odometrija A): samo 24 % točaka unutar 2 cm,
  medijan 57 mm. To je metrička karta koju bi nacrtala konfiguracija A i ujedno pokazuje zašto
  konfiguracije D i E poravnavaju skenove sa zidovima.

U profilu realistic (sjeme 1) T2, T3 i M2 su uspješni. **T1 završava sigurnosnim zaustavljanjem
`closed_straight_guard`.** Uzrok je šum prednjeg senzora blizine: zaštita koja provjerava očekivano
mjesto kočenja ima marginu od 15 mm, a šum na ~1 m ima σ ≈ 12 mm. To je zadatak 13b-6 (robusnost FSM-a
na šum senzora blizine). Bez tog popravka završni testovi u profilu realistic nisu valjani.

## 13b-6 – robusnost navigacije na šum i otvorene dijelove labirinta

**Polazno stanje.** Offline batch (`python -m experiment.batch`): A × {ideal, realistic} × T1–T3,
M1–M5 × 3 sjemena, oznaka `base13b6`.

| | T1 | T2 | T3 | M1 | M2 | M3 | M4 | M5 |
|---|---|---|---|---|---|---|---|---|
| ideal | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0/3 | 3/3 |
| realistic | 0/3 | 3/3 | 3/3 | 2/3 | 1/3 | 0/3 | 0/3 | 0/3 |

Uočena su dva navigacijska uzroka neuspjeha. Oba su neovisna o procjeni položaja, pa bi kvarila
usporedbu konfiguracija.

1. **`closed_straight_guard` (samo uz šum).** Kod raskrižja sa zatvorenim ravnim smjerom zaštita
   predviđa mjesto kočenja iz **jednog** očitanja prednjeg senzora u referentnoj točki. Pretpostavljeni
   šum na 0.55 m je σ ≈ 7.5 mm, a margina 15 mm.

   **Popravak (`navigation_fsm.py`, `navigation_config.py`).** Udaljenost u referentnoj točki je
   **medijan zadnjih 15 očitanja** iz ravne vožnje. Svako očitanje se odometrijom prenese u referentnu
   točku: d_k − (s_ref − s_k), uzimaju se samo uzorci iz zadnjih 0.10 m. Medijan smanjuje σ na ≈ 2 mm.
   Margina ostaje 15 mm. U idealnom profilu rezultat je praktički isti.
2. **Sudar u M3 (i M5 sa šumom): kriva os hodnika u HOLD načinu.** Micromouse labirinti imaju duge
   dijelove s jednom otvorenom stranom. Tamo vođenje nema sva četiri zida pa drži zapamćenu os hodnika.
   Os se pomiče za zadani kut okreta, a robot se zbog gubitka u okretu okrene malo manje. Bez FULL
   mjerenja ta se razlika zbraja: u M3 je narasla na 5.7° i robot je bočno udario u zid.

   **Popravak (`corridor_guidance.py`): vođenje po jednom zidu (LEFT / RIGHT).**
   - Kad bočni i dijagonalni senzor jedne strane vide običan zid, (y, α) se računa samo iz te strane.
     Koristi se model po senzoru dobiven iz 12 769 FULL uzoraka vožnje uživo u T2:
     d_R = 0.1842 + 0.9942 y + 0.1711 α i d_FR = 0.3397 + 1.2310 y + 0.6311 α, te simetrično za lijevo.
   - Mjerenje mora biti valjano 5 koraka zaredom, a α se mora slagati sa zapamćenom osi unutar 8°.
     Tako se odbacuju očitanja na krajevima zidova.
   - Vođenje je isto u svim konfiguracijama A–E i ne dira procjenu položaja. Mijenja samo fizičku
     putanju robota.

**Nakon popravka** (oznaka `fsm13b6`, isti skup vožnji):

| | T1 | T2 | T3 | M1 | M2 | M3 | M4 | M5 |
|---|---|---|---|---|---|---|---|---|
| ideal | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | **3/3** | 0/3 | 3/3 |
| realistic | **3/3** | 3/3 | 3/3 | **3/3** | **3/3** | **3/3** | 0/3 | **3/3** |

* Nema više navigacijskih neuspjeha.
* M4 ne uspijeva zbog pogreške odometrije: karta ima duplikate, pa cilj nije u karti. To je stvarni
  rezultat konfiguracije A i upravo ono što fuzija treba popraviti.
* T1–T3 u idealnom profilu imaju iste rezultate kao prije (vremena, pogreške). Popravak ne mijenja
  ponašanje tamo gdje je bilo dobro.
* Najmanji razmak od zida u M5 porastao je s 4 mm na 17 mm.
* Profil realistic sada daje gotovo iste rezultate kao ideal. Šum senzora blizine više ne utječe na
  navigaciju, nego samo na vođenje.

Testovi: `test_step08_corridor_guidance.TestSingleWall`. Model jednog zida vraća (y, α) unutar 4 mm i
1°. S osi krivom za 4° robot u 4 m s jednom otvorenom stranom ostaje unutar 3 mm od sredine, a bez
vođenja po jednom zidu odluta više od 10 cm.

**Provjera uživo (9. 10. 2026.): PASS u oba slučaja koji su offline prije padali.**

| | T1, realistic, s1 | M3, ideal |
|---|---|---|
| prije popravka (offline) | sigurnosno zaustavljanje | sudar |
| uživo, misija S4 | **uspjeh** | **uspjeh** |
| karta | točna (23/23, 22 brida) | točna (33/33, 37 bridova, 5 petlji) |
| najmanji razmak od zida | 15 mm | 17 mm |
| istraživanje / vožnja do cilja | 510 s / 84 s | 798 s / 248 s |
| omjer rute prema optimalnoj | 1.036 | 1.024 |
| najveća pogreška u raskrižju | 958 mm, 10.1° | 500 mm, 5.1° |
| offline predviđanje (istraživanje, najveća pogreška) | 475 s, 1115 mm | 738 s, 690 mm |
| QA LiDAR-a: medijan / p90 | 1.2 / 5.7 mm | 1.2 / 5.7 mm |

* Startna kalibracija žiroskopa pogađa izvučeni pomak nule −18.857 °/s s procjenom −18.853 °/s.
* Pogreška konfiguracije A raste s duljinom vožnje: u T1 sa šumom robot do cilja stiže s pogreškom
  procjene od gotovo 1 m i 10°. Cilj ipak nalazi jer se raskrižja pridružuju po poznatim hodnicima.
  To je mjera koliko je A blizu granice.

## 13b-3 – sve metrike iz zapisa vožnje (`evaluation/metrics.py`)

Metrike iz 13e računaju se **nakon** vožnje iz njezina zapisa:

    python -m evaluation.metrics ../results/runs/T1_A_realistic_s1      (jedna vožnja -> metrics.json)
    python -m evaluation.metrics --all                                   (sve vožnje bez metrics.json)

`run.py` ih izračuna automatski na kraju vožnje. Petlja uživo više ne računa ništa iz GT-a (razmak
tijela od zida, QA LiDAR-a): radi samo ono što radi robot. Kod je numpy i vektoriziran, pa jedna vožnja
od 1000 s traje oko 4 s.

| oznaka | izračun |
|---|---|
| S2 | sudar iz simulatora ILI razmak tijela ≤ 0 (9 točaka obrisa robota prema stvarnim zidovima, svaki korak) |
| L1, L2 | RMSE, maksimum i konačna vrijednost kroz sve korake |
| L3 | središte raskrižja u karti robota prema stvarnom čvoru |
| L4 | konačna pogreška položaja / prijeđeni put [%]; najveća pogreška kuta / broj okreta |
| L5 | NEES; prazno za A (nema kovarijance) |
| T1–T3 | iz evaluacije karte; T3 = zatvorene petlje / petlje labirinta |
| T4 | rezerva zatvaranja = R − najveća udaljenost pridruživanja MATCH |
| K1 | točke LiDAR-a po **procijenjenoj** pozi prema stvarnim zidovima (RMSE, medijan, udio unutar 2 cm). Usput: senzor po GT pozi (provjera kalibracije) |
| P1 | udaljenost osi od središnje linije hodnika uz koji robot vozi (samo dijelovi paralelni smjeru vožnje, samo vožnja ravno) |
| P2 | kut prema osi hodnika = najbližem višekratniku 90° |
| P3 | idealna putanja = središnje linije kroz iste stvarne čvorove istim redom (u START-u točka okreta pred zidom). Daje srednje i najveće odstupanje te omjer duljina |
| P4 | stvarni kut nakon svakog okreta prema rešetki od 90° |
| P5 | samo okreti u raskrižjima (LEFT/RIGHT): os robota prema središtu raskrižja na početku okreta. U slijepoj ulici robot se okreće pred zidom, ne u središtu |
| P6 | RMS dω/dt u vožnji ravno; broj promjena predznaka naredbe ω po metru |
| P7 | najmanji i srednji razmak; udio vremena ispod 50 mm |
| P8 | ∫(\|u_L\|+\|u_R\|)/2 dt po metru puta |
| R1 | `aggregate()` preko više vožnji (stopa uspjeha, srednja vrijednost i σ glavnih metrika) |

**Vožnje uživo od 9. 10. (konfiguracija A):**

| metrika | T1 realistic s1 | M3 ideal |
|---|---:|---:|
| L1 položaj RMSE / max / kraj [mm] | 362 / 958 / 958 | 198 / 500 / 152 |
| L2 kut RMSE / max [°] | 5.88 / 11.51 | 2.50 / 6.79 |
| L3 središta raskrižja srednje / max [mm] | 280 / 666 | 124 / 306 |
| T3 petlje, T4 rezerva | 0/0, – | 5/5, 0.042 m |
| K1 karta LiDAR-a RMSE, unutar 2 cm | 220 mm, 12 % | 113 mm, 17 % |
| P1 središnja linija RMS / max [mm] | 6.0 / 21.0 | 6.2 / 15.9 |
| P2 kut prema osi RMS / max [°] | 0.70 / 1.69 | 0.69 / 1.70 |
| P3 idealna putanja srednje / max [mm], omjer | 3.3 / 32.8, 1.020 | 3.2 / 30.8, 1.019 |
| P4 okreti RMS / max [°] | 1.00 / 1.61 | 1.00 / 1.70 |
| P5 točka okreta RMS / max [mm] | 17.5 / 27.1 | 18.6 / 24.8 |
| P6 dω/dt RMS, promjene predznaka / m | **2.83 rad/s², 64** | 0.59 rad/s², 1.6 |
| P7 razmak min / srednji, < 50 mm | 14.7 / 170 mm, 11.7 % | 17.0 / 169 mm, 14.1 % |
| P8 napor | 1.76 | 1.73 |

* Navigacija je vrlo točna i jednaka u oba slučaja: robot vozi 6 mm RMS od središnje linije, a put je
  samo 2 % dulji od idealnog.
* M3 ima samo 42 mm rezerve pri zatvaranju petlje. Petlja je zatvorena, ali gotovo na granici
  radijusa.
* **Nalaz P6:** sa šumom senzora blizine naredba ω trza (64 promjene predznaka po metru prema 1.6 bez
  šuma). Putanja je i dalje ista (P1 6 mm), jer je trzanje brže nego što robot može slijediti. Ali
  stvarni robot bi mjerenja vođenja filtrirao. Prijedlog: niskopropusni filtar na ŷ i α̂ u vođenju,
  isti za sve konfiguracije, prije završnih testova.

## 13b-8 – filtar mjerenja u vođenju i robusno mirovanje

**Problem (nalaz P6 iz 13b-3):** sa šumom senzora blizine naredba ω trzala je ~55 promjena predznaka
po metru. Putanja je bila dobra, ali takvo upravljanje ne bi bilo prihvatljivo na stvarnom robotu.

**Filtar (`navigation/corridor_guidance.py`):** prediktivni niskopropusni filtar na ŷ (pomak od osi)
i α̂ (kut prema osi), isti za sve konfiguracije:

* predikcija iz odometrije u svakom koraku: α̂ += Δθ, ŷ += Δs·sin α̂ (filtar ne kasni za stvarnim
  gibanjem, nego samo prigušuje šum mjerenja);
* korekcija mjerenjem s pojačanjem dt/(τ+dt), τ = `MEAS_TAU_S` = 0,05 s;
* reset pri promjeni načina (FULL / LEFT / RIGHT / HOLD), da se ne miješaju mjerenja različitih zidova.

**Usput nađena greška (FSM, `navigation/navigation_fsm.py`):** u M5 realistic s1 robot je nakon
kočenja ostao zauvijek u POSITION/BRAKE. Lijevi kotač stajao je na rubu jednog impulsa enkodera i
izmjenjivao +1/−1 impuls po koraku (naredba PI regulatora unutar mrtve zone motora), pa uvjet
„20 koraka s točno 0 impulsa” nikad nije bio ispunjen. Novi uvjet mirovanja (BRAKE i SETTLE):

* u zadnjih 20 koraka svaki |impuls| ≤ 1 i neto zbroj po kotaču |Σ| ≤ 1 (`STOP_COUNT_TOL`): treperenje
  je mirovanje, a sporo puzanje (−1 u svakom koraku, Σ = −20) nije;
* sigurnosna granica `STOP_TIMEOUT_S` = 3 s; broj takvih slučajeva upisuje se u `summary.json`
  (`stop_timeouts`). U cijelom batchu je 0.

**Provjera (offline batch `filt13b8` prema `fsm13b6`, T1 T2 T3 M1 M2 M3 M5, sjeme 1–2, konfiguracija A):**
S4 28/28 (prije M5 realistic s1 nije završio).

| prosjek 14 vožnji | ideal prije | ideal poslije | realistic prije | realistic poslije |
|---|---:|---:|---:|---:|
| P1 centralna linija RMS [mm] | 6,58 | 6,53 | 6,97 | 6,86 |
| P2 kut prema osi RMS [°] | 0,80 | 0,82 | 0,82 | 0,85 |
| P4 točnost okreta RMS [°] | 1,03 | 1,11 | 1,06 | 1,07 |
| P5 točka okreta RMS [mm] | 17,3 | 17,5 | 18,1 | 18,3 |
| P6 dω/dt RMS [rad/s²] | 0,61 | 0,59 | **2,85** | **0,73** |
| P6 promjene predznaka [1/m] | 1,14 | 1,08 | **54,7** | **2,29** |
| P7 min. razmak [mm] | 19,9 | 19,7 | 19,1 | 20,1 |
| P8 napor pogona [|u|·s/m] | 1,58 | 1,56 | 1,58 | 1,56 |

Šumom uzrokovano trzanje je nestalo (P6 ×4 manje, oscilacije ×24 manje), a praćenje osi hodnika (P1,
P2) i okreti (P4, P5) su praktički isti. Ostaje provjera uživo (T1 realistic).

## 13b-7 – brzina simulacije

LiDAR se iscrtava samo svaki 10. korak (10 Hz, kao stvarni URG-04LX), a ne u svakom koraku od 10 ms.
Skripta skenera koristi explicit handling, a bridge šalje svaki novi sken jednom. Mjerenje je
napravljeno s `checks.step13b_speed` (robot stoji 20 s, s uključenim 3D prikazom).

| | prije | poslije |
|---|---:|---:|
| faktor stvarnog vremena | 0.69 | **1.69** |
| simulator po koraku od 10 ms | 14.35 ms | 5.87 ms |
| Python po koraku | 0.04 ms | 0.05 ms |

Iscrtavanje LiDAR-a u svakom koraku trošilo je oko 60 % vremena simulatora. Simulacija sada radi brže
od stvarnog vremena. Provjera `step13b_live_sensors` i dalje daje PASS: IMU je u svakom paketu,
LiDAR stiže na 10 Hz s 684 zrake i sken je iz istog koraka kao IMU.

## 13c – stvarni testni labirinti (`mazes/micromouse.py`, `data/mazes/`)

Stvarni labirinti iz javne zbirke micromouse natjecanja i treninga
(github.com/micromouseonline/mazefiles, commit 548dcdeb, vidi `data/mazes/SOURCE.md`), pretvoreni u
FusionBot rešetku (ćelija 18 cm → 1.085 m, topologija zidova točno ista; START = startna ćelija,
ROOT = prvi čvor sjeverno od nje). Registrirani kao M1–M7 uz vlastite T1–T3, pa ih
`python -m sim.maze_scene M4` gradi u CoppeliaSimu. Pregled: `docs/figures/step13c_mazes.png`.

| ID | labirint | težina | čvorovi | petlje | hodnik (ćelija) |
|---|---|---|---|---|---|
| M1 | training 5×5 (minimaze) | lako | 18 | 1 | 24 |
| M2 | training 10×5-a | srednje | 25 | 6 | 51 |
| M3 | training 10×5 test1 | srednje-teško | 32 | 5 | 50 |
| M4 | practice 8×8-a | teško | 51 | 6 | 84 |
| M5 | training 8×8 test2 | vrlo teško | 52 | 14 | 81 |
| M6 | UK natjecanje 2016, finale (16×16) | iznimno teško | 136 | 32 | 286 |
| M7 | All Japan 2017, expert finale (16×16) | iznimno teško | 174 | 5 | 259 |

Prvi offline rezultati (MazeSim, konfiguracija A, DFS do kraja):

| labirint | idealna odometrija | A s live gubitkom u okretu (0.47°/0.66°) |
|---|---|---|
| M1 | ispravno, 38 događaja, 383 s | ispravno |
| M2 | ispravno, 62 događaja, 677 s | ispravno |
| M3 | ispravno, 74 događaja, 732 s | **sudar (SAFETY_ABORT) nakon 67 događaja** |
| M4 | ispravno, 114 događaja, 1188 s | **karta pogrešna: 1 duplikat, zatvorene 4/6 petlji** |
| M5 | ispravno, 132 događaja, 1229 s | **karta pogrešna: 14 duplikata, zatvoreno 10/14 petlji** |
| M6, M7 | offline simulacija prespora (> 10 min) – treba ubrzati ray-cast | – |

Stvarni labirinti dakle jasno razdvajaju konfiguracije: A u težim labirintima griješi, što je
upravo ono što fuzija treba popraviti.

## 13d – misija: istraživanje → START → cilj (`navigation/mission.py`)

Svaki labirint ima START i CILJ (slika `docs/figures/step13c_mazes.png`; natjecateljski labirinti:
označeni centar; trening labirinti bez oznake: čvor najdalji od starta po hodnicima). Misija:

1. **istraživanje** (DFS) cijelog labirinta – završava povratkom na START;
2. ako je istraživanje uspjelo (`complete`): **plan najkraćeg puta u vlastitoj topološkoj karti**
   (Dijkstra, duljina brida = udaljenost procijenjenih središta raskrižja) do raskrižja najbližeg
   zadanoj poziciji cilja (≤ 0.55 m) i **vožnja do cilja**; svaki dolazak se i dalje pridružuje karti;
3. kraj: `goal_reached`, ili uredan kraj s razlogom (`explore_budget`, `explore_inconsistent`,
   `goal_not_in_map`, `goal_lost`, `inconsistent`).

Cilj je zadan kao **pozicija** (kao u micromouse natjecanju), pa koje je raskrižje „cilj“ ovisi o
procjeni poze konfiguracije – kriva karta može dovesti na krivo raskrižje, što evaluacija (GT) otkriva.
Metrike vožnje do cilja: uspjeh (GT čvor = cilj), vrijeme, duljina puta prema optimalnoj (GT).

Offline (konfiguracija A, s live gubitkom u okretu): T2, T3, M1, M2, M4, M5 – cilj postignut, put
0.3–0.7 m dulji od optimalnog (zaustavljanje na događaju i polukružni okret na startu).
Živo: `python -m checks.step11_live_dfs --maze M1` (misija je uključena; `--no-goal` samo istražuje).

## 13e – metrike uspješnosti, šum i smetnje po konfiguracijama

### Metrike (svaka vožnja; GT samo za evaluaciju)

| skupina | metrika | jedinica | definicija |
|---|---|---|---|
| **misija** | S1 istraživanje završeno | da/ne | DFS završio `complete` i vratio se na START |
| | S2 bez sudara | da/ne | nijedan sudar tijela sa zidom (GT); uz to najmanji razmak tijelo–zid [mm] |
| | S3 cilj postignut | da/ne | robot na kraju stoji na stvarnom čvoru cilja (GT) |
| | **S4 misija uspješna** | da/ne | S1 ∧ S2 ∧ S3 |
| **lokalizacija** | L1 pogreška pozicije | m | RMSE, max i konačna ‖(x̂,ŷ) − (x,y)‖ kroz cijelu vožnju |
| | L2 pogreška kuta | ° | RMSE, max i konačna \|θ̂ − θ\| |
| | L3 pogreška u čvorovima | m | srednja razlika procijenjenog i stvarnog položaja središta raskrižja |
| | L4 drift | % / °·okret⁻¹ | pogreška pozicije po prijeđenom metru; pogreška kuta po okretu |
| | L5 konzistentnost (EKF) | – | NEES: udio koraka unutar 95 % χ² granice (B–E) |
| **topološka karta** | T1 karta ispravna | da/ne | nema duplikata ni pogrešnih spajanja, svi bridovi ispravni |
| | T2 duplikati / pogrešna spajanja | broj | čvorovi labirinta koji su u karti više puta / različiti čvorovi spojeni u jedan |
| | T3 zatvorene petlje | broj / očekivano | prepoznata zatvaranja petlje prema broju petlji labirinta |
| | T4 rezerva zatvaranja | m | R − najveća udaljenost pridruživanja (R = 0.373 m); manja rezerva = bliže promašaju |
| **metrička karta** | K1 točnost zidova | mm | RMSE udaljenosti LiDAR točaka karte od stvarnih zidova |
| | K2 podudarnost mreže | % | udio ispravno zauzetih/slobodnih ćelija occupancy grida prema GT mreži (korak 14) |
| **učinkovitost** | E1 vrijeme istraživanja | s | simulirano vrijeme do povratka na START |
| | E2 vožnja do cilja | s, m | vrijeme i duljina puta START → cilj |
| | E3 omjer rute | – | duljina vožnje do cilja / optimalna duljina (GT); 1 = optimalno |
| **putanja (kvaliteta gibanja)** | P1 odstupanje od središnje linije | mm | RMS i max bočnog odmaka osi robota od središnje linije hodnika (GT) |
| | P2 odstupanje kuta od osi hodnika | ° | RMS i max kuta robota prema smjeru hodnika u vožnji ravno |
| | P3 odstupanje od idealne putanje | m, – | srednja i najveća udaljenost vožene putanje od idealne (središnje linije iste rute); omjer duljina vožene / idealne |
| | P4 točnost okreta | ° | pogreška stvarnog kuta nakon svakog okreta prema višekratniku 90° (GT) |
| | P5 točnost točke okreta | mm | udaljenost točke okreta od središta raskrižja (GT) |
| | P6 glatkoća | rad/s², broj | RMS promjene kutne brzine u vožnji ravno; broj oscilacija upravljanja |
| | P7 sigurnosni razmak | mm, % | najmanji i srednji razmak tijelo–zid; udio vremena ispod 50 mm |
| | P8 napor pogona | – | ∫\|u\|dt naredbi motora po metru puta |
| **robusnost** | R1 stopa uspjeha | % | udio uspješnih misija preko sjemena šuma (Monte Carlo) |
| | R2 granica otkaza | razina | najmanja razina smetnje pri kojoj misija više ne uspijeva |

Glavni pokazatelj uspješnosti navigacije je **S4** (preko sjemena: **R1**); pokazatelji kvalitete
su L1/L2 i T1; učinkovitost E1–E3.

### Šum po konfiguracijama (profil `realistic`)

Šum djeluje na fizički senzor, dakle na sve što ga koristi (i na vođenje):

| izvor šuma | utječe na procjenu poze u | utječe na vođenje u |
|---|---|---|
| kvantizacija enkodera (1024 CPR) | A, B, C, D, E (put; kut u A, B, D) | – |
| senzori blizine (σ = 2 mm + 1 %) | B, E (PROX-LOC) | A–E |
| žiroskop MPU-6050 (šum, bias, faktor skale) | C, E | – |
| LiDAR URG-04LX (±30 mm / ±3 %) | D, E | – (karta u A–E) |

### Smetnje (korak 19) – iste za sve konfiguracije, po razinama

Smetnja je fizička promjena svijeta ili kvar, ne šum. Ubacuje se kontrolirano i ponovljivo; svaka
pogađa slabost određenog senzora, a očekuje se da je druga konfiguracija ispravi:

| ID | smetnja | razine | pogađa | očekivano ispravljaju |
|---|---|---|---|---|
| D1 | sklizak pod – gubitak kuta u okretu ×k | k = 2, 3, 5 | kut iz enkodera (A, B, D) | žiroskop (C, E), LiDAR (D, E) |
| D2 | pogrešan polumjer kotača (trošenje) | ±1 %, ±2 % | put iz enkodera (svi) | PROX-LOC (B, E), LiDAR (D, E) |
| D3 | pogrešan efektivni razmak kotača | ±2 %, ±5 % | kut iz enkodera (A, B, D) | žiroskop (C, E), LiDAR |
| D4 | proklizavanje u vožnji ravno (udari) | 1 %, 3 % puta | put i kut iz enkodera | LiDAR (D, E), PROX-LOC (B, E) |
| D5 | skok biasa žiroskopa | +0.2, +0.5 °/s usred vožnje | žiroskop (C, E) | ZUPT (C, E), LiDAR (E) |
| D6 | degradacija LiDAR-a (ispali/lažni povrati) | 20 %, 50 % zraka | LiDAR (D, E) | robusno poravnanje; E ima ostale |
| D7 | kvar senzora blizine (jedan bočni ne radi) | lijevi / desni | vođenje (svi), PROX-LOC (B, E) | LiDAR (D, E) |

Plan: svaka smetnja × A–E × profil `realistic` × odabrani labirint (T3, M4) × 3 sjemena;
izlaz je R2 (granica otkaza) i usporedba L1/L2/T1/S4 s nominalnom vožnjom.
