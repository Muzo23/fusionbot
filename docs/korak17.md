# Korak 17 – konfiguracije D (LiDAR) i E (sve)

**Zahtjev zadatka:** fuzija podataka iz više izvora; modeliranje nepoznatog okruženja.

## D: poravnanje LiDAR skena s vlastitom kartom (`localization/scan_match.py`)

* Bez pretpostavke o mreži labirinta (za razliku od B): sken se poravnava s robotovom VLASTITOM kartom
  zauzetosti (korak 14).
* Polje udaljenosti (likelihood field) u prozoru 6 × 6 m oko robota, do 0,30 m; obnavlja se svakih
  10 skenova ili kad se robot pomakne 1,5 m (oktagonalna dilatacija, ~2 ms).
* Gauss-Newton s Huberovim težinama (4 cm) na ~170 zraka; točke bez zida u blizini (novi hodnik,
  prepreka) ne sudjeluju. Kovarijanca = varijanca ostatka × (JᵀWJ)⁻¹ × COV_INFLATE; u ravnom hodniku
  smjer duž osi gotovo nema informacije (velika varijanca).
* EKF korekcija z = (x, y, θ) s χ² gateom. Korekcija > 0,25 m ili > 8° nije poklapanje.
* Karta se ažurira NAKON korekcije (na ispravljenoj pozi).
* **COV_INFLATE = 1000** (podešeno offline T1 realistic): 9 → L1 179 mm, NEES 11 574; 100 → 80 mm / 313;
  1000 → 48–63 mm / 22–36; 10 000 → 132 mm / 32. Uzrok: zrake jednog skena i skenovi na 10 Hz prema istim
  zidovima su jako korelirani, a karta potječe iz iste procjene. Kandidat za sustavno podešavanje (15b).
* Isprobana i varijanta „split” (apsolutna korekcija samo prema zidovima starijim od 30 s + varijanca
  karte, inače samo srednja vrijednost): T1 L1 309 mm → odbačena (`SCAN_MODE = "plain"`).

## E: sve zajedno

Žiroskop (kut) + ZUPT + PROX-LOC + LiDAR. Ne treba novi kod (ista jezgra).

## Prvi rezultati (offline, T1 realistic s1, s LiDAR-om)

| | L1 RMSE | L2 RMSE | NEES | K1 | K2 |
|---|---:|---:|---:|---:|---:|
| A | 340 mm | 5,75° | 0,93 | 218 mm | 59,6 % |
| D | 63 mm | 1,34° | 35,5 | 49 mm | 83,9 % |
| E | 3 mm | 0,15° | 1,02 | 17 mm | 89,3 % |

Batch `lid17` (A–E × T1–T3, M1, M2, M3, M5 × 2 sjemena, realistic, s LiDAR-om) pokrenut; rezultati:
`python -m evaluation.compare --tag lid17` (ako batch nije dovršen: ponoviti naredbu iz sažetka).
