# Korak 5 – Bridge prema CoppeliaSimu (lockstep)

Python i CoppeliaSim razmjenjuju po jedan paket stanja i jedan paket komandi svakih 10 ms
preko ZeroMQ (PUB/SUB, protokol 2.0, float64).

Simulator (Lua, skripta na /FusionBot):
- sysCall_actuation: upisuje zadnje komande kotača u svojstva zglobova (čita ih motor, korak 1)
- sysCall_sensing: čita pozicije zglobova, 5 proximity senzora (filtrirano na okolinu, bez robota),
  GT pozu (samo evaluacija), zastavice fizičkog i vizualnog sudara; šalje paket stanja k i u
  lockstep načinu čeka komandu izračunatu baš za stanje k.

Posljedica: kašnjenje je uvijek točno 1 korak (10 ms), neovisno o opterećenju računala; isti ulaz
daje bit-identičan run. Simulacija nije vezana uz stvarno vrijeme pa radi brže od njega.

Robusnost: ako komanda ne stigne u 20 ms, isto stanje se šalje ponovno (Python ga prepoznaje kao
duplikat i ponavlja istu komandu); nakon 10 s simulacija javlja grešku. Na kraju Python šalje
RELEASE prije bilo kojeg Remote API poziva, jer Remote API poslužuje ista nit koju lockstep blokira.

Lua bridge je identičan verziji validiranoj u FusionBot2 testu 13a (dva runa bit-identična,
kašnjenje točno 1 korak, ~1.5× brže od stvarnog vremena).

Live test koraka 5 (checks/step05_live_drive.py) provjerava korake 1–5 zajedno: odziv kotača na skok,
vožnju 1 m (odometrija vs GT), zavoj +90° (odometrija vs GT) i determinizam.
