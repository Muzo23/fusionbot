# Korak 14 – metrička karta: occupancy grid i metrika K2

**Zahtjev zadatka:** „modeliranje nepoznatog okruženja”. Uz topološku kartu (MazeMemory, korak 10)
robot sada gradi i metričku kartu zauzetosti. Ona je ujedno podloga za dinamičke scenarije (14b) i za
poravnanje LiDAR skena u konfiguracijama D i E.

## Kako radi (`mapping/occupancy_grid.py`)

* Mreža ćelija 2 cm (hodnik 0,729 m = 36 ćelija; šum LiDAR-a ±30 mm kao 2σ).
* Gradi se **online**, u svakoj konfiguraciji jednako, iz svakog LiDAR skena (10 Hz) na **procijenjenoj**
  pozi robota. Razlika između konfiguracija je samo u kvaliteti procjene poze, pa K2 mjeri učinak fuzije.
* Log-odds zapis (Thrun, Burgard, Fox: *Probabilistic Robotics*, pogl. 9, inverzni model senzora):
  ćelija kroz koju zraka prolazi dobiva L_FREE = −0,40, ćelija odraza L_OCC = +0,85.
  Zrake bez odraza čiste prostor do 4 m (specificirani doseg URG-04LX).
* Vrijednost je ograničena na [−4, 4]. Zato se karta može promijeniti: zasićeno slobodna ćelija postaje
  zauzeta nakon 6 pogodaka (0,6 s), zasićeno zauzeta se oslobađa nakon 9 prolaza zrake (0,9 s). To je
  potrebno za privremene prepreke (14b).
* Stanja: zauzeta p > 0,62 (jedan pogodak), slobodna p < 0,43 (jedan prolaz), inače nepoznata.
* Robot ne zna veličinu labirinta: mreža počinje oko START-a (±3 m) i raste u blokovima od 2 m.
* Trošak: oko 4 ms po skenu (0,4 % vremena na 10 Hz). Karta se sprema u `map_grid.npz`.

## Metrika K2 (`evaluation.metrics`, nakon vožnje)

Usporedba s pravim labirintom (GT samo za evaluaciju), za poznate ćelije (slobodne ili zauzete):

| | definicija |
|---|---|
| K2 podudarnost | ispravne poznate ćelije / sve poznate ćelije |
| zauzete na zidu | zauzeta ćelija je ispravna ako je unutar 3 cm od pravog zida |
| slobodne u slobodnom prostoru | slobodna ćelija je ispravna ako joj je središte u slobodnom prostoru (ili unutar 3 cm od zida) |
| pokrivenost | udio stvarno slobodnih ćelija (dalje od 3 cm od zida) koje karta ima kao slobodne |

Uz metriku se crta `map_grid.png` (karta + pravi zidovi crveno), slika za rad.
Za vožnje snimljene prije koraka 14 karta se rekonstruira iz `scans.npz` istim kodom.

## Provjera

* Testovi `tests/test_step14_grid.py` (7): jedna zraka, zrake bez odraza, rast mreže, pojava i nestanak
  prepreke (≤ 6 / ≤ 9 skenova), spremanje, K2 = 100 % uz točnu pozu, K2 < 90 % uz pogrešku poze
  10 cm / 3°.
* Offline s LiDAR-om, konfiguracija A:

| vožnja | K1 RMSE | K2 podudarnost | zauzete na zidu | pokrivenost |
|---|---:|---:|---:|---:|
| T2 ideal s1 | 129 mm | 78,5 % | 22,8 % | 91,3 % |
| T1 realistic s1 | 218 mm | 59,6 % | 11,5 % | 81,3 % |

Karta po GT pozi daje 100 %, pa je sav pad posljedica pogreške procjene poze konfiguracije A
(odometrija: zakret karte nekoliko stupnjeva, „duplicirani” zidovi). To je polazna točka koju fuzija
(B–E) mora popraviti.

---

# Korak 14b – dinamičko okruženje (prepreke koje nisu u karti)

**Zahtjev zadatka:** „dinamičko i kompleksno simulacijsko okruženje”. Labirint se nakon istraživanja
mijenja: u vožnji do cilja pojavljuju se prepreke koje robot nije vidio pri izradi karte.

## Scenariji (`mazes/dynamic.py`)

| | prepreka | očekivano ponašanje |
|---|---|---|
| DYN1 | predmet 0,15 × 0,15 m prelazi hodnik ispred robota brzinom 0,20 m/s (osoba, drugi robot) | stati, pričekati, nastaviti |
| DYN2 | zatvorena vrata preko cijelog hodnika, 8 s | čekati dok se ne otvore, nastaviti |
| DYN3 | predmet 0,40 × 0,10 m uz jedan zid | proći pored; bočni senzor vidi „zid” 10 cm bliže nego u karti |
| DYN4 | ista vrata, trajno zatvorena | hodnik označiti blokiranim, okrenuti se, isplanirati drugi put |

* Prepreke se pojavljuju tek u vožnji do cilja. Okidač je stvarna (GT) pozicija robota: robot je u
  tom hodniku, vozi prema prepreci i udaljen je do 1,2 m (vrata), 1,3 m (DYN1) ili 2 m (DYN3).
  To je događaj okruženja; robot ga ne zna.
* Smještaj: na rutu koju je robot upravo isplanirao (njegova raskrižja preslikana u stvarne čvorove u
  kojima ih je vidio), sredina brida, nikad u raskrižju i nikad na prvom bridu. DYN4 samo na bridu bez
  kojeg do cilja i dalje postoji put, duljem od 2,16 m, 1,6 m od raskrižja: robot se mora zaustaviti i
  okrenuti u hodniku, da bi raskrižje iza sebe ponovno vidio na uobičajen način.
* Scenarij ALL: DYN3, DYN1, DYN2 i DYN4 na četiri različita brida rute, DYN4 zadnji.
* Iste definicije koriste offline svijet (`DynamicWorld`, ray-cast senzora blizine i LiDAR-a, sudar),
  živa scena (`sim/dynamic_scene.py`) i evaluacija (`dynamic.json`).

## Kako robot prepoznaje prepreku (`navigation/mission.py`, `navigation_fsm.py`)

* Za vrijeme istraživanja misija bilježi **odometrijsku duljinu svakog brida** (od središta do središta
  raskrižja, iz odometrije u trenutku događaja + kanonski pomak do središta).
* U vožnji do cilja misija za brid kojim robot vozi daje **donju granicu očitanja FRONT senzora**:
  preostali put do sljedećeg raskrižja + pola hodnika − položaj senzora (+ jedan razmak mreže ako je
  ravno otvoreno). Ne ovisi o apsolutnoj procjeni poze, nego samo o odometriji na tom bridu.
* FSM: očitanje FRONT ≤ 0,25 m i barem 0,15 m kraće od te granice, 3 uzastopna koraka → stanje
  **BLOCKED**: kočenje, mirovanje, čekanje.
  * FRONT slobodan 1 s → nastavak (DYN1, DYN2).
  * Nakon 15 s i dalje blokirano → misija označi brid blokiranim (`MazeMemory.turn_back`), Dijkstra
    bez tog brida od raskrižja iza robota → okret 180° u hodniku → povratak (pridruživanje `RETURN`)
    → nova ruta. Ako drugog puta nema: kraj `goal_blocked`.
* Otkrivanje koristi samo FRONT senzor blizine, jednako u svim konfiguracijama A–E. Fuzija utječe
  preko točnosti procjene i karte (gating u EKF-u, korak 15+).

## Metrike (`evaluation.metrics`)

DYN: postavljene i susretnute prepreke, broj zaustavljanja, nastavaka i okreta, **lažna zaustavljanja**
(nema prepreke unutar 1 m od robota), trajanja čekanja, najmanji razmak tijela do prepreka. Prepreke
ulaze i u S2/P7 (razmak tijela). P5 broji samo okrete u raskrižjima (okret zbog prepreke ne).

## Uživo

Bridge protokol 2.2: paket naredbe može nositi položaje prepreka; bridge ih postavlja prije koraka.
Runner prije vožnje stvori 8 skrivenih kvadara `FB_MAZE_dyn_<i>` (statični, nisu respondable,
narančasti), a nakon vožnje ih ukloni. Potrebna je ponovna instalacija bridgea.

## Provjera (offline, konfiguracija A, scenarij ALL)

`tests/test_step14b_dynamic.py` (16 testova): geometrija i okidači prepreka, smještaj, ray-cast,
blokirani brid i povratak u memoriji, planiranje oko blokiranog brida, FSM (očekivani zid nije
prepreka, jedno šumno očitanje se ignorira, prepreka nestane → nastavak, ostane → okret, nema puta →
kraj), cijela vožnja M3 sa sve četiri prepreke.

Statički batch `static14b` (T1–T3, M1, M2, M3, M5, realistic, 2 sjemena): S4 14/14, zaustavljanja zbog prepreke 0 (nema lažnih).

Batch `ALL_dyn14b` (M1, M2, M3, M5 × ideal, realistic × 3 sjemena): **S4 24/24**, lažnih zaustavljanja
0, najmanji razmak do prepreka 94–106 mm. M1 nema brid za DYN4 (2 zaustavljanja, 2 nastavka); M2
nakon okreta zbog DYN4 nova ruta ne prolazi pored DYN2.
