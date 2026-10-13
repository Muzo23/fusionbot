# Korak 9 – Navigacijski automat stanja (FSM)

FSM slaže vožnju kroz labirint od raskrižja do raskrižja, 100 Hz. Ulazi su proximity percepcija
(korak 6), primitivi gibanja (korak 7) i vođenje po hodniku (korak 8). FSM ne procjenjuje pozu, ne
čita GT, ne upravlja motorima izravno i ne pamti kartu. Odluku na raskrižju donosi vanjski
DecisionProvider: u ovom koraku pravilo lijeve ruke, a od koraka 11 DFS s memorijom.

## Stanja

    INIT → FOLLOW ⇄ APPROACH → (događaj + odluka) → POSITION → TURN → EXIT → FOLLOW …
                                         ├─ STRAIGHT → EXIT
                                         ├─ FINISH   → FINISHED
                                         └─ nedopuštena odluka → SAFETY_ABORT

**Događaji opažanja** (trenutni; odluka se donosi u istom koraku):
- **SIDE_CONFIRMED:** bočni otvor je CONFIRMED i nijedna strana više nije CANDIDATE. Ravno je
  otvoreno ako FRONT u tom trenutku ne vidi zid.
- **DEAD_END:** FRONT_BLOCKED bez potvrđenog bočnog otvora. Robot staje odmah.

**POSITION** (za LEFT, RIGHT i UTURN):
- **ADVANCE, ravno otvoreno:** vožnja D_TURN_ENTRY = 0.529676 m od reference, pa kočenje. Os robota
  stane u središtu raskrižja.
- **ADVANCE, ravno zatvoreno:** vožnja do FRONT_BLOCKED, uz zaštitu (vidi K2 niže).
- **BRAKE:** 20 uzastopnih uzoraka bez impulsa enkodera.
- **HOLD:** 0.5 s mirovanja.

**TURN:** okret MotionControllerom (korak 7), zatim 20 uzoraka bez impulsa. Os hodnika u vođenju
pomakne se za kut okreta.

**EXIT:** vožnja ravno s vođenjem (HOLD na osi). Semantika otvora je zamrznuta dok LEFT i RIGHT ne
vide zid 5 uzastopnih uzoraka. Tada reset_openings() i ponovna obrada istog očitanja (ono može već
pokazivati CANDIDATE sljedećeg raskrižja).

**Sigurnost** (ima prednost pred svime):
- vanjski događaj (kontakt u simulatoru);
- FRONT_SAFETY: vožnja naprijed uz FRONT < 0.030 m;
- FRONT_BLOCKED tijekom EXIT-a (neočekivano).

Pri svakoj promjeni primitiva FSM traži reset PI regulatora kotača (nalaz iz koraka 7).

## Promjene u odnosu na FusionBot2 (revizija)

- **Jedna brzina, 0.25 m/s.** Svi pragovi (zaustavni put kod FRONT_BLOCKED, D_TURN_ENTRY, mjerni
  model hodnika, kompenzacija trenja) validirani su na toj brzini. Nema više brzine krstarenja ni
  predkočenja, pa nestaje i problem prijelaza APPROACH→FOLLOW. Veća brzina krstarenja je budući rad.
- **FRONT_SAFETY** (K5) – fizička zadnja linija zaštite.
- **EXIT prati FRONT.**
- **Vođenje po hodniku** radi u svim stanjima vožnje naprijed (FOLLOW, APPROACH, POSITION, EXIT).
- **Opažanje i odluka** u istom koraku; eksplicitni FINISH.

### K2 – zaštita kod zatvorenog ravnog smjera

FusionBot2 je imao fiksnu zaštitu na 0.529676 m od reference, uz očekivani FRONT_BLOCKED na
0.512151 m (margina 17.5 mm).

Offline nalaz u koraku 9 (T1, 24 zatvorena raskrižja): FRONT_BLOCKED se aktivira precizno, 16–18 mm
prije središta. Referentni događaj (SIDE CONFIRMED) pomiče se pak za ≈ (W/2)·tan α s kutom robota u
trenutku reference: 516–541 mm prije središta, odnosno ±12 mm pri kutu od 1.4°. Okidač je zato bio
na 499.9–525.1 mm, a u najgorem slučaju margina je pala na 4.6 mm.

Novo rješenje: u trenutku reference FRONT senzor već vidi daleki zid raskrižja (po tome se zna da je
ravno zatvoreno), pa je preostali put poznat za svaki događaj posebno:

    očekivani okidač = d_front(referenca) − 0.057581
    zaštita          = očekivani okidač + 15 mm

Offline odstupanje od očekivanog je +0.0 do +2.6 mm na svim raskrižjima T1, T2 i T3, neovisno o
kutu robota.

## Offline provjera

Offline simulator labirinta (tests/maze_sim.py):
- model pogona s rotacijskim trenjem identificiranim u koraku 8;
- zidovi iz src/mazes (isti algoritam kao generator scene);
- zrake senzora s montažama iz kalibracije 8a;
- provjera kolizije obrisom robota.

Petlja ima isti redoslijed kao runtime: mjerenje → FSM → PI → primjena.

Rezultati s pravilom lijeve ruke:
- **T1:** svih 44 događaja (cijelo stablo, povratak na ROOT, polukružni okret iza STARTA) poklapaju
  se s obilaskom grafa labirinta (čvor, dostupni smjerovi L/S/R i odluka).
- **T2 i T3:** isto vrijedi za 20 odnosno 16 događaja.
- **Kolizije:** nijedna; najmanji razmak tijela od zida 19–23 mm (polukružni okreti u slijepim
  ulicama).

## Live provjera

Pravilo lijeve ruke kroz cijeli T1 (44 događaja, ≈ 8 min simuliranog vremena). Kriteriji:
- svi događaji se poklapaju s obilaskom;
- završno stanje FINISHED;
- bez kolizije;
- |okidač − očekivano| < 10 mm;
- razmak tijela > 5 mm.

### Live rezultat (5. 10. 2026) – PASS

| mjera | vrijednost |
|---|---|
| događaji | 44 / 44 se poklapaju s obilaskom, zatim FINISH na ROOT-u |
| zatvorena raskrižja | 24; okidač − očekivano: +0.2 … +2.4 mm (zaštita 15 mm) |
| najmanji razmak tijela od zida (GT) | 12.4 mm (offline 20.9 mm) |
| kolizija | nema |
| vrijeme | 526 s simulirano / 280 s stvarno |

**Razmak po stanjima.** Najmanji razmak je u okretima (TURN:TURNING), konkretno u polukružnim
okretima u slijepim ulicama I, START, H i F: 12.4–13.5 mm. Nominalno je razmak pri polukružnom okretu
22 mm (polovica širine hodnika 0.364 m minus radijus okretanja 0.342 m). Live robot pri okretu klizne
oko 10 mm (pomak osi iz koraka 7), pa je 12.4 mm očekivano. U vožnji ravno razmak je ≥ 49 mm.

**Drift smjera odometrije (konfiguracija A).** Na kraju obilaska odometrija − GT = +11.2°. Drift raste
jednoliko, ≈ +1° na 50 s:

| t [s] | 51 | 101 | 151 | 201 | 251 | 301 | 351 | 401 | 451 | 501 |
|---|---|---|---|---|---|---|---|---|---|---|
| drift [°] | 1.0 | 1.7 | 2.5 | 4.2 | 6.5 | 5.1 | 5.8 | 7.3 | 8.1 | 9.4 |

Fizički robot cijelo vrijeme ostaje poravnat s hodnicima, jer ga vođenje drži uz zidove. Procjena
smjera iz odometrije pak odlazi, zbog gubitka od ≈ 0.6° po okretu (korak 7). Karta i položaji
raskrižja procijenjeni samo iz odometrije bit će zato zakrenuti do ≈ 11°. To je izravna motivacija za
fuziju (C: žiroskop, D/E: LiDAR, B: PROX-LOC) i ulaz za korak 10: prepoznavanje već posjećenog
raskrižja prema procijenjenoj pozi mora biti otporno na takav drift.
