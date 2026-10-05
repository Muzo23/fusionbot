# Korak 10 – Topološka memorija labirinta (MazeMemory)

MazeMemory sprema istraženi labirint kao graf:
- **čvorovi** su raskrižja: procijenjeno središte i izlazi po globalnim smjerovima;
- **bridovi** su hodnici, s brojem prolazaka.

Memorija ne bira manevre (to je DFS u koraku 11), ne čita senzore i ne koristi GT.

## Odluka (5. 10. 2026, opcija 1)

Svaki događaj raskrižja smješta se prema **procjeni poze konfiguracije** (A: odometrija; B–E: fuzija).
Pouzdanost prepoznavanja već viđenog raskrižja tako ovisi o kvaliteti procjene poze. To je mjerljiv
rezultat rada: točnost topološke karte po konfiguraciji.

## Geometrija

- **Okvir:** odometrija se inicijalizira u poznatoj početnoj pozi (START), pa se okvir procjene
  poklapa s okvirom labirinta do na pogrešku procjene.
- **Smjerovi:** E = +x, N = +y, W = −x, S = −y. Procijenjeni smjer robota zaokružuje se na najbliži
  glavni smjer (granica 45°; live drift u koraku 9 bio je 11°).
- **Središte raskrižja:**

      (x, y) + D·(cos θ̂, sin θ̂),   D = 0.530606 m (bočni otvor) ili 0.018455 m (slijepa ulica)

## Prepoznavanje

Događaj se pridružuje spremljenom raskrižju ako vrijede oba uvjeta:
1. središta su bliže od R = 0.373 m (polovica između nominalnog rasipanja 0.204 m i pola razmaka
   čvorova);
2. skup izlaza je isti (topološki potpis).

Ako više kandidata zadovoljava uvjete, bira se najbliži. Inače nastaje novo raskrižje. Odbacivanja
zbog strukture bilježe se zasebno.

## Bridovi

depart(smjer) otvara brid; sljedeći događaj ga zatvara na raskrižju dolaska i povećava broj
prolazaka. Zatvaranje petlje je dolazak na poznato raskrižje kroz izlaz koji još nije bio povezan.

## Provjera

**Offline (FSM + pravilo lijeve ruke, memorija na odometriji iz offline modela):**

| labirint | raskrižja | bridovi | duplikati | lažna spajanja | zatvorene petlje |
|---|---|---|---|---|---|
| T1 | 23 (22 + START) | 22 točna | 0 | 0 | – |
| T2 | 10 | 10 točnih | 0 | 0 | 1 |
| T3 | 8 | 8 točnih | 0 | 0 | 1 |

**Live podaci (ponovna obrada 45 događaja iz koraka 9, poza odometrije rekonstruirana iz loga):**

| poza | raskrižja | duplikati | max. udaljenost prepoznavanja | ispravno |
|---|---|---|---|---|
| GT | 23 | – | 0.236 m* | da |
| odometrija (A) | 25 | R, START | 0.332 m | ne |

\* Slijepa ulica iza STARTA je 0.236 m iza početne poze (čvor START), a ne pogreška.

Udaljenost prepoznavanja s odometrijom raste s vremenom:

| događaj | 3 (U) | 16 (D) | 20 (R) | 37 (P) | 41 (S) |
|---|---|---|---|---|---|
| udaljenost [m] | 0.022 | 0.159 | 0.314 | 0.142 | **0.332** |

Na događaju 42 (povratak na ROOT nakon ≈ 500 s) pogreška pozicije iz odometrije iznosi ≈ 0.65 m, uz
drift smjera ≈ 11°. Raskrižje se više ne prepoznaje, pa se R i START spremaju dvaput.

Zaključak za rad: u konfiguraciji A topološka karta T1 postaje neispravna nakon ≈ 500 s istraživanja,
a s točnom pozom ostaje ispravna. Rezerva prepoznavanja (R − udaljenost) je mjerna veličina za
usporedbu konfiguracija A–E.

Testni podaci: tests/data/step09_live_events.csv i tests/data/step09_live_event_poses.csv (poza
odometrije i GT u trenutku svakog događaja).

## 10b – Topološka karta: spremanje i crtanje (mapping/graph_map.py)

MazeMemory je sama topološka karta: čvorovi su raskrižja na PROCIJENJENIM središtima, bridovi su
prijeđeni hodnici.

- **graph_to_dict / save_graph:** JSON po pokretanju s raskrižjima, bridovima i svim događajima
  (udaljenost prepoznavanja, zatvaranje petlje, odbacivanja zbog strukture).
- **draw_graph:** crtež grafa, po želji preko stvarnih zidova i s oznakama stvarnih čvorova (GT,
  samo za evaluaciju). Duplikati su crveni, a debljina brida raste s brojem prolazaka.

checks.step10_graph_figure crta kartu iz live pokretanja koraka 9 dvaput, s GT pozom i s odometrijom.
S odometrijom se vidi zakret cijele desne grane (drift ≈ 11°) i dvostruki R i START.

Metrička karta (occupancy grid iz LiDAR-a) je korak 14. Obje karte zajedno su izlaz istraživanja
nepoznatog okruženja.
