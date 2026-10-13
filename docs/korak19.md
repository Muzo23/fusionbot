# Korak 19 – smetnje (D1–D7) i razina šuma (N1, N2)

**Zahtjev zadatka:** stohastičke smetnje – sustavni šum i nelinearni vanjski utjecaji na kretanje; precizna
procjena stanja u uvjetima nesigurnosti; fuzija podataka iz više izvora (koja konfiguracija ispravlja koju smetnju).

## Šum i smetnja – razlika

* **Šum** (profil `realistic`, korak 13a) je u svakoj vožnji: slučajna pogreška svakog očitanja prema tablicama
  podataka (žiroskop, LiDAR, senzori blizine, kvantizacija enkodera).
* **Smetnja** je fizička promjena svijeta ili kvar koji šum ne pokriva: sustavna pogreška modela, vanjski
  utjecaj na kretanje ili kvar senzora. Robot za nju ne zna: filtar i navigacija zadržavaju parametre iz 15 / 15b.
* **N1 / N2** su skeniranje razine šuma: koliko lošije senzore sustav podnosi.

## Smetnje (`src/experiment/disturbances.py`)

| ID | smetnja | tip | kada | razine | ubacuje se | pogađa |
|---|---|---|---|---|---|---|
| D1 | sklizak pod – gubitak kuta u okretu ×k | nelinearni vanjski utjecaj | cijela vožnja, svaki okret | ×2, ×3, ×5 | model pogona (samo offline) | kut iz enkodera |
| D2 | pogrešan polumjer kotača (trošenje) | sustavna pogreška modela | cijela vožnja | ±1 %, ±2 % | kutovi kotača → enkoderi | put iz enkodera |
| D3 | pogrešan efektivni razmak kotača | sustavna pogreška modela | cijela vožnja | ±2 %, ±5 % | kutovi kotača → enkoderi | kut iz enkodera |
| D4 | proklizavanje u vožnji ravno (udari) | stohastički vanjski utjecaj | slučajno, Poisson 1 udar/m, 0,2 s | 1 %, 3 % puta | model pogona (samo offline) | put i kut iz enkodera |
| D5 | skok biasa žiroskopa | nagli sustavni kvar | sredina istraživanja | +0,2, +0,5 °/s | model žiroskopa | C, E |
| D6 | degradacija LiDAR-a (pola ispada, pola lažnih kratkih povrata) | stohastički kvar senzora | sredina istraživanja | 20 %, 50 % zraka | sken | D, E |
| D7 | pomak kalibracije bočnog senzora +20 mm | kvar senzora | sredina istraživanja | lijevi / desni | očitanje senzora | vođenje, PROX-LOC |

* **Sredina istraživanja:** prvi takt vožnje (FOLLOW) nakon što robotova karta sadrži polovicu raskrižja
  labirinta (⌈čvorovi/2⌉). Robot tada već ima dobru kartu i procjenu, pa se vidi otkriva li fuzija promjenu i
  oporavlja li se.
* **D2 / D3** se primjenjuju na kutove kotača prije enkodera (srednji dio × (1+e₂), diferencijalni × (1+e₃)).
  To je ekvivalentno stvarnom polumjeru r/(1+e₂) i razmaku B(1+e₃); regulatori kotača vide iste brzine kao
  s istrošenim kotačem. Radi jednako uživo i offline. e > 0: enkoderi precjenjuju put / okret.
* **D4:** udari su slučajni (vlastiti generator, sjeme vožnje + 190 000), isti za sve konfiguracije. Jedan kotač
  proklizava (1 + 0,25) puta više od drugog, pa svaki udar daje i mali zakret.
* **D7:** odabrana opcija 1 (pomak kalibracije, ne potpuni kvar). Potpuni kvar bočnog senzora FSM bi vidio kao
  otvor sa strane i ruši navigaciju u svim konfiguracijama (test FSM-a, ne fuzije).
* **D1 i D4 su samo offline** (mijenjaju fiziku pogona); uživo bi trebalo mijenjati trenje u sceni.

## Razina šuma (`sensors/noise_models.py`, `NoiseProfile(scale=...)`)

| ID | što | razine | konfiguracije | labirint |
|---|---|---|---|---|
| N1 | σ svih senzora × k | ×0,5, ×2, ×4, ×8 | A–E | T3, M4 |
| N2 | σ samo jednog senzora × k (žiroskop / LiDAR / blizina) | ×2, ×4, ×8 | relevantne | T3 |

Množe se slučajni dijelovi (bijeli šum, lutanje i skala žiroskopa; σ LiDAR-a i senzora blizine); kvantizacija i
početni pomak nule žiroskopa (uklanja ga početna kalibracija) ostaju. Isto sjeme daje iste slučajne brojeve, pa
se razlikuje samo veličina šuma.

## Plan pokusa (`python -m experiment.step19`)

* realistic, T3 i M4, sjemena 1–3; N2 samo T3; svaka razina × relevantne konfiguracije + nominalna vožnja.
* Samo konfiguracije na koje smetnja može djelovati (D5: C, E; D6: D, E; N2gyro: C, E; N2lidar: D, E).
  Za ostale bi vožnja bila identična nominalnoj.
* LiDAR se ray-casta samo za D i E (A–C ga ne koriste; K metrike nisu dio koraka 19).
* 729 vožnji (321 s LiDAR-om); procjena oko 13 h na 2 jezgre. Runner je nastavljiv: gotove vožnje se preskaču.
* Izlaz: `results/batches/step19_<tag>.csv` i `--report` → tablica (S4, karta, L1, L2, ANEES po smetnji,
  razini i konfiguraciji) i **R2 – granica otkaza**: najmanja razina (po redu jačine) na kojoj uspješnost misije
  padne ispod nominalne.

```
cd fusionbot\src
python -m experiment.step19 --dry-run
python -m experiment.step19 --tag s19
python -m experiment.step19 --tag s19 --report
```

## Oporavak od kvara biasa (nova zaštita u EKF-u, `ekf.ZUPT_RECOVERY`)

Prva provjera D5 pokazala je da χ² gate **sprječava oporavak**: skok 0,5 °/s daje inovaciju ZUPT-a
NIS = (0,5 / 0,05)² = 100 ≫ 6,6, pa se svaki ZUPT odbaci do kraja vožnje (T3 C: karta potpuno kriva,
186 događaja umjesto 20).

Zaštita: bias se proglašava nepoznatim (σ 1 °/s, korelacije obrisane) i sljedeći ZUPT-ovi ga ponovno nauče,
ali samo ako su ispunjena sva tri uvjeta:
1. 20 ZUPT-ova zaredom je odbijeno;
2. odbijene inovacije su međusobno konzistentne (σ ≤ 3 × šum žiroskopa), dakle stalni pomak, a ne gibanje;
3. robot je u **dubokom mirovanju**: 0,5 s bez ijednog impulsa enkodera.

Treći uvjet je nužan. U nominalnim vožnjama gate s pravom odbija oko 28 % ZUPT-ova: test mirovanja
(≤ 1 impuls u 0,2 s) propušta do 0,3 °/s stvarnog zakretanja dok se tijelo smiruje nakon okreta. To je sporo
i glatko, pa prolazi i test konzistentnosti. Prva inačica bez trećeg uvjeta lažno je resetirala bias 1–2 puta
po vožnji i udvostručila L1 za C (replay 21 snimke: validacija 86 → 172 mm). S trećim uvjetom nominalne
vožnje su identične onima bez zaštite (L1 182,4 / 85,9 mm, ANEES 2,39 / 1,57).

Rezultat (T3 C s1):
* 0,5 °/s: jedno resetiranje, bias točno naučen (−18,33 °/s, stvarno −18,35), misija uspješna.
* 0,2 °/s: bez resetiranja, ZUPT ga nauči postupno.

U oba slučaja kut do prvog dubokog mirovanja odluta 6–9°, pa karta ima duplikate. Za rad: **C ovisi o
zaustavljanjima**, a E bi kvar trebao premostiti drugim izvorima. `ZUPT_RECOVERY = False` daje stanje bez
zaštite (ablacija).

## Nalaz: D4 i sigurnosna zaštita FSM-a

Proklizavanje 3 % (T3 A s1) prekida vožnju sigurnosnom zaštitom `closed_straight_guard`: FSM uspoređuje
odometrijsku udaljenost s očekivanim mjestom zida (rezerva 17,5 mm). Kotači proklizavaju, pa odometrija
„prestigne” robota. Navigacija koristi samo enkodere, ne fuziju, pa D4 pogađa **sve konfiguracije jednako**.
To je ograničenje arhitekture (navigacija neovisna o procjeni poze), a ne fuzije; navodi se u radu. Moguće
poboljšanje (izvan opsega): zaštita koristi fuziranu udaljenost.

## Prve provjere (T3 s1)

| | rezultat |
|---|---|
| A D1 ×3 | karta kriva (duplikati), istraživanje „inconsistent” – kao predviđanje iz koraka 13 |
| A D2 +2 % | karta točna, L1 258 mm, NEES 9,6 |
| A D3 +5 % | karta kriva |
| A D4 3 % | SAFETY_ABORT (vidi gore) |
| C D5 0,5 °/s | bez zaštite: karta kriva, 186 događaja, istraživanje „inconsistent”; sa zaštitom: 28 događaja, bias naučen, misija uspješna, karta s duplikatima |
| B D7 L | karta točna, L1 4 mm, NEES 28 (pomak kalibracije pristrano pomiče PROX-LOC) |

Provjera runnera (T3 s1, D i E, tag `smoke`):

| | D: L1 / ANEES | E: L1 / ANEES | S4 |
|---|---:|---:|---|
| nominalno | 60 mm / 286 | 3 mm / 2,7 | uspjeh |
| D6 50 % | 151 mm / 501 | 5 mm / 2,9 | uspjeh |
| D7 L | 58 mm / 259 | 6 mm / 67 | uspjeh |
| D7 R | 61 mm / 283 | **270 mm / 10 182** | uspjeh |
| N1 ×4 | 163 mm / 1 369 | **209 mm / 2 576** | uspjeh |
| N1 ×8 | 121 mm | 297 mm | neuspjeh: `front_safety` (senzori blizine ×8 – navigacija) |

## Nalaz: „zaključavanje” E uz samouvjereni LiDAR (D7 R, N1 ×4)

U E s pristranim bočnim senzorom (D7 R) PROX-LOC malo pomakne procjenu, a karta zauzetosti se gradi na
pomaknutoj procjeni. Kovarijanca ostaje oko 3 mm jer je poravnanje skena samouvjereno (ANEES D-a u 15b:
140–600). Kad se pomak nakupi, χ² gate počne odbacivati **ispravne** apsolutne korekcije PROX-LOC-a
(994 odbijenih, nominalno 0), pa se procjena „zaključa” uz vlastitu kartu i odluta do 0,9 m (T3 s1, oko 245 s).

Gate i samouvjereni izvor zajedno daju zarobljavanje: nekonzistentnost D-a iz 15b u E postaje opasna tek
kad se pojavi kvar. Za rad je to važan rezultat. Moguća poboljšanja (odluka):
1. isti mehanizam kao za bias: N uzastopno odbijenih apsolutnih korekcija (PROX-LOC) dok LiDAR prolazi
   → povećati kovarijancu poze i pustiti apsolutni izvor da je vrati;
2. ostaviti kako jest i opisati kao ograničenje (rezultat cijelog batcha pokazat će koliko je često).


## Testovi

`tests/test_step19_disturbances.py` (14): D2 / D3 preslikavanje kutova (i preko ±π), identitet bez smetnje,
okidanje D5–D7 na sredini istraživanja (samo u vožnji), D6 udio, D4 izgubljeni udio puta, skaliranje šuma
(iste slučajne vrijednosti, σ × k), oznake, plan (samo relevantne konfiguracije, LiDAR samo D/E), oporavak od
skoka biasa (sa zaštitom naučen, bez nje blokiran, mala promjena bez resetiranja). Ukupno 226 testova, svi prolaze (nakon izmjene zaštite biasa ponovljeni testovi koraka 15–19: 46, svi prolaze).
