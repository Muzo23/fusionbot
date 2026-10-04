# FusionBot – diplomski rad

Povećanje pouzdanosti autonomne navigacije primjenom metoda fuzije podataka u kompleksnim okruženjima.
Varijanta zadatka: autonomno istraživanje i modeliranje nepoznatog okruženja (labirint).

Projekt se gradi korak po korak. Svaki korak: koncept → skripta → test → commit.

## Struktura (cilj)

```
src/
  core/          robot_config, motor, encoder, wheel PI          (koraci 0–3)
  localization/  odometrija, PoseEstimator, EKF                   (4, 15–18)
  sim/           bridge (lockstep), transport, generator scene   (5, 12)
  perception/    proximity čitanje i semantika otvora            (6)
  navigation/    MotionController, vođenje hodnikom, FSM,
                 MazeMemory, MazeDecision (DFS)                   (7–11)
  mapping/       occupancy grid, metrike karte                    (14)
  experiments/   runtime, runner, rekorder, smetnje, Monte Carlo (13, 19–20)
tests/           test_stepXX_*.py, jedan po koraku
docs/            bilješke koraka (podloga za poglavlje 3 rada)
results/         izlazi runova (nije u gitu)
```

## Pravila

1. GT (ground truth) nikad nije ulaz regulacije ni estimatora; samo evaluacija.
2. Svaka fizička konstanta postoji na točno jednom mjestu (`core/robot_config.py`).
3. Parametri se ne podešavaju da bi jedan test prošao; prvo se karakterizira mehanizam.
4. Isti navigacijski stack u svim konfiguracijama A–E; razlikuje se samo estimator poze.

## Testovi

Iz korijena repozitorija:

```
python -m unittest discover -s tests -v
```
