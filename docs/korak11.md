# Korak 11 – Autonomno istraživanje: DFS nad MazeMemory

## Dorada memorije: pridruživanje po poznatom bridu

U koraku 10 svaki se dolazak prepoznavao po položaju. Na tvojim live podacima to je u konfiguraciji A
nakon ≈ 500 s dalo dvostruki ROOT i START, iako se robot vratio hodnikom koji već poznaje. Opcija 1 je
zamišljena drukčije: pri povratku se DFS oslanja na graf, a prepoznavanje po položaju služi samo za
zatvaranje petlje. Memorija sada radi tako:

| dolazak | pridruživanje | ovisi o procjeni poze? |
|---|---|---|
| poznatim bridom (oba kraja već spremljena) | drugi kraj tog brida (EDGE) | ne; udaljenost se samo bilježi |
| novim bridom | po položaju + topološki potpis (MATCH ili NEW) | da – zatvaranje petlje |

Pridruživanje po bridu s udaljenošću > R označava se kao nekonzistentno. Tako drift ostaje vidljiv,
a da pritom ne kvari kartu.

Stari način (samo po položaju) ostaje kao opcija edge_association=False, radi usporedbe u radu. Na
istim live podacima:

| varijanta | T1 karta | duplikati | nekonzistentna pridruživanja |
|---|---|---|---|
| samo po položaju (korak 10) | neispravna | R, START | – |
| po poznatom bridu (korak 11) | ispravna | – | 3 (događaji 42–44, udaljenost 0.55–0.90 m) |

## DFS (Trémaux) – navigation/maze_decision.py

Na svakom događaju raskrižja primjenjuju se pravila redom:
1. **Zatvaranje petlje:** dolazak novim hodnikom na poznato raskrižje (MATCH) → povratak istim
   hodnikom (UTURN).
2. **Istraživanje:** postoji izlaz bez brida → kreni njime (prednost LEFT, STRAIGHT, RIGHT).
3. **Povratak:** sve je istraženo → izađi izlazom kojim je raskrižje PRVI put dosegnuto (DFS
   roditelj).
4. **FINISH:** povratak na START (čvor 0, slijepa ulica iza početne poze) kad više nema ništa
   neistraženo. Robot tamo staje.

Zaštita: najveći broj događaja (budget) → FINISH s razlogom „budget”. Razlog svake odluke se bilježi.

## Offline rezultati

Simulator labirinta; „live klizanje” = klizanje u okretima izmjereno live u koraku 7: tijelo se
okrene ≈ 0.6° manje na 90° nego što kaže odometrija, što odgovara driftu konfiguracije A.

| labirint | idealna odometrija | odometrija s live klizanjem (A) |
|---|---|---|
| T1 (stablo) | ispravno, 43 događaja + FINISH na STARTU, 472 s | ispravno; 4 nekonzistentna pridruživanja |
| T2 (dvije manje petlje) | ispravno, obje petlje zatvorene, 272 s | ispravno, obje petlje zatvorene |
| T3 (prsten 5.4 × 5.4 m) | ispravno, petlja zatvorena, 299 s | **petlja promašena**: R se ne prepozna, prsten se obiđe još jednom, 7 čvorova dvostruko, 638 s |

Zaključak za rad: u konfiguraciji A topološka karta stabla ostaje ispravna, jer se povratak
pridružuje po bridu. Dugačka petlja (T3) se pak ne zatvara, jer pogreška poze iz odometrije na kraju
prstena premaši radijus prepoznavanja. Robot zato labirint istraži dvaput i karta je neispravna. To
mora popraviti fuzija (B–E).

U offline simulator dodan je parametar turn_slip (tests/plant.py), kalibriran prema koraku 7.

## Live provjera (T1)

checks.step11_live_dfs: robot sam istraži T1 s procjenom poze iz odometrije (A), vrati se na START i
stane. Na kraju se sprema i crta topološka karta (graph_T1_A.json / .png).

Kriteriji:
- FINISH s razlogom „complete” na čvoru 0;
- 43 događaja jednaka obilasku T1;
- memorija ispravna (bez duplikata i lažnih spajanja, svi bridovi točni);
- bez kolizije.

T2 i T3 live dolaze nakon generatora scene (korak 12).

### Live rezultat (5. 10. 2026) – PASS

| mjera | vrijednost |
|---|---|
| završetak | FINISH „complete” na J0 (START) |
| događaji | 43 + FINISH, svi jednaki obilasku T1 |
| memorija | 23 raskrižja, 22 brida, bez duplikata i lažnih spajanja |
| pridruživanja | 22 po poznatom bridu, 2 nekonzistentna (R: 0.577 m, START: 0.902 m) |
| zatvorena raskrižja | okidač − očekivano +0.2 … +2.4 mm |
| najmanji razmak tijela | 12.6 mm, bez kolizije |
| vrijeme | 515 s simulirano / 288 s stvarno |

Slika: figures/step11_graph_T1_A_live.png. Desna grana je zakrenuta zbog drifta odometrije, ali je
topologija potpuno točna.

**Ponovljivost:** svi događaji dogodili su se u istom simuliranom trenutku kao u koraku 9 (4.20 s,
20.07 s, … 500.46 s), uz isti put robota. Promijenio se samo donositelj odluka (pravilo lijeve ruke →
DFS), a odluke su u stablu iste. Lockstep sustav je dakle potpuno determinističan, što je preduvjet za
uparene Monte Carlo usporedbe konfiguracija (korak 20).
