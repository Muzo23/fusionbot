# Korak 7 – MotionController (primitivi gibanja)

Sloj između navigacijske logike (FSM, korak 9) i regulatora kotača (korak 3). Primitiv gibanja
pretvara se u referencu tijela (v_ref, ω_ref), a ta se preko kinematike diferencijalnog pogona
pretvara u reference brzina kotača.

Primitivi:
- IDLE: v = 0, ω = 0
- STRAIGHT(v): v = konst., ω = 0 (u koraku 8 ω zamjenjuje vođenje po hodniku)
- TURN(Δ): okret u mjestu za relativni kut Δ, zatvorena petlja po ODOMETRIJI:

      φ_acc ← φ_acc + Δθ_odo                 (svakih 10 ms, bez svođenja na (−π, π])
      e = Δ − φ_acc
      ω_ref = sat(K·e, ±ω_max),  K = 2.0 s⁻¹,  ω_max = 1.2 rad/s
      kraj: |e| ≤ 0.33° kroz N = 5 uzastopnih uzoraka

Zašto akumulacija bez svođenja: +180° i −180° su različiti manevri (CCW i CW). DFS pri povratku
mora ponoviti točno suprotan okret da se robot ne bi okretao "kraćim putem" i dodirnuo zid.

Zašto P-regulator sa zasićenjem: većinu okreta robot vozi konstantnom ω_max, a u blizini cilja brzina
linearno pada (K·e) pa robot ne prebaci. Uvjet od 5 uzoraka sprječava prerani kraj zbog
kvantizacije enkodera. Jedan inkrement kuta iznosi 2π·r/(1024·B_eff) ≈ 0.055°, pa je tolerancija od
0.33° približno 6 inkremenata.

Zastavica `completed` je True točno na jednom uzorku. Nakon toga kontroler prelazi u IDLE.

Važan nalaz (offline, potvrđuje se live): pri početku svakog primitiva treba resetirati PI regulatore
kotača. Bez reseta integrator tijekom smirivanja integrira puzanje ispod rezolucije enkodera, pa
preostali integral za vrijeme okreta pomakne os robota za ≈ 5 mm. S resetom pomak iznosi ≈ 0.1 mm.
Pravilo za runtime (korak 13): promjena primitiva ⇒ reset obaju PI regulatora.

Offline rezultati (model pogona iz koraka 1–4, J = 0.004 kg·m²):

| zadano | stvarno | odometrija | trajanje |
|-------:|--------:|-----------:|---------:|
|   ±90° | ±89.746° |  ±89.718° |  2.83 s |
|  ±180° | ±179.768° | ±179.765° | 4.16 s |

Live provjera (T1): robot vozi od STARTA do središta raskrižja ROOT (1.422634 m po odometriji), pa
izvodi okrete +90, −90, +180 i −180.

Kriterij za KONTROLER (relativno na odometriju, jer kontroler ne može biti bolji od svog senzora):
- |odometrija − cilj| ≤ 0.6°
- trajanje ≤ 5 s
- predznak okreta očuvan
- nema kolizije

Live run 1 (5. 10. 2026) – kontroler PASS, odometrija točno kao offline:

| zadano | odometrija | GT | odo − GT | GT/odo | trajanje | pomak osi |
|------:|-----------:|---:|---------:|-------:|---------:|----------:|
| +90° | +89.718° | +89.356° | +0.362° | 0.9960 | 3.30 s | 8.1 mm |
| −90° | −89.718° | −88.762° | −0.956° | 0.9893 | 3.22 s | 8.8 mm |
| +180° | +179.765° | +179.173° | +0.591° | 0.9967 | 4.64 s | 14.2 mm |
| −180° | −179.765° | −178.662° | −1.103° | 0.9939 | 4.70 s | 14.3 mm |

Ostali rezultati:
- Ravna vožnja: odometrija 1.4234 m, GT 1.4232 m (+0.2 mm).
- Nakon četiri okreta neto GT smjer je +1.105°.

Analiza: GT se tijekom smirivanja mijenja samo ±0.04°, dakle pogreška nastaje tijekom okreta. Kotači
proklizavaju i robot se okrene manje nego što enkoderi pokazuju. Postoji i asimetrija CW/CCW. U ovom
nizu svaki okret mijenja smjer u odnosu na prethodni, pa se učinak smjera ne može odvojiti od učinka
promjene smjera (zakretanje kotačića, promjena smjera vrtnje kotača). Os robota klizi 8–14 mm po
okretu, a neto pomak nakon niza je ≈ 39 mm. To je stvarni pomak, a ne odmak referentne točke GT-a.
Offline model pogona nema proklizavanje, pa ga nije mogao predvidjeti.

Zaključak za rad: to je sustavna pogreška odometrije u okretima (≈ 0.4–1.1 %), upravo ona koju
fuzija (žiroskop u C, LiDAR u D/E) treba ispraviti. Konfiguracija A se ne "dotjeruje" asimetričnim
korekcijama. Opravdana je samo simetrična kalibracija mjerila (B_eff, po uzoru na UMBmark).

Korak 7b (checks.step07b_turn_calibration) razdvaja učinke kroz dva pokretanja:
- A: +90 ×4, zatim −90 ×4
- B: +180 ×2, zatim −180 ×2

Okreti se razvrstavaju u FIRST (prvi nakon ravne vožnje), REPEAT (isti smjer kao prethodni) i
REVERSAL (suprotan smjer). Iz okreta REPEAT predlaže se kalibrirani B_eff (srednja vrijednost CW i
CCW). Preostala asimetrija se dokumentira kao nemodelirana pogreška.

Rezultat koraka 7b (5. 10. 2026, bez ograničenja ubrzanja):

| okreti | srednja \|odo−GT\| | GT/odo |
|-------:|-------------------:|-------:|
| 90° (8 okreta) | 0.61° | 0.991–0.996 |
| 180° (4 okreta) | 0.55° | 0.995–0.998 |

Zaključci:
- **Pogreška ne raste s kutom.** Okret od 180° ima jednaku ili manju pogrešku od okreta od 90°, dakle
  to nije pogreška mjerila (B_eff). Model je stalna pogreška po okretu od ≈ 0.6° (robot se okrene
  manje), uz raspršenje ±0.15°. Prijedlog skripte B_eff = 0.34139 m ne prihvaća se: ispravio bi
  okrete od 90°, a okrete od 180° pretjerano ispravio, na ≈ −0.6°. B_eff ostaje 0.3392 m.
- **Gdje nastaje pogreška (CSV):** u prvih 0.5 s okreta odometrija prestigne GT za ≈ 2.0°, jer kotači
  proklizavaju pri skokovitom ubrzanju na ω_max. Do kraja konstantne brzine razlika naraste na
  ≈ 2.4°, a tijekom kočenja se većim dijelom vrati. Ostatak je ≈ 0.6°. Pogreška je dakle prijelazna
  pojava (ubrzanje), a ne geometrijska.
- **Asimetrija:** CW je ≈ 0.15–0.3° lošiji od CCW. Okret nakon promjene smjera nije lošiji od
  ponovljenog, pa zakretanje kotačića nije uzrok.
- **Za EKF (korak 15):** šum procesa u okretu σ_θ ≈ 0.6° po okretu, uz sustavni dio i nemodelirani
  ostatak.

Eksperiment: ograničenje kutnog ubrzanja u TURN, α_max = 2.4 rad/s² (opcija `alpha_max_rad_s2`).
Iznos odgovara usporavanju koje P-regulator ionako ima pri ω_max (K·ω_max = 2.4 rad/s²), pa su
ubrzanje i usporavanje simetrični. Offline okret od 180° traje 4.42 s umjesto 4.16 s. Ako live
pokaže manju pogrešku po okretu, α_max postaje zadana vrijednost.

Rezultat pokusa s α_max = 2.4 rad/s² (5. 10. 2026): rampa NE pomaže.

| | bez rampe | s rampom |
|---|---:|---:|
| srednja \|odo−GT\|, 90° | 0.61° | 0.64° |
| srednja \|odo−GT\|, 180° | 0.55° | 0.61° |
| trajanje okreta od 180° | 4.6 s | 4.9 s |

Usporedba u vremenu pokazuje uzrok:
- I s rampom razlika odo−GT pri konstantnoj ω = 1.2 rad/s naraste na ≈ 2.5°, a pada kad ω pada. To je
  zaostajanje tijela koje ovisi o brzini (meki kontakt kotač–pod u MuJoCo), a ne posljedica naglog
  ubrzanja.
- Ostatak od ≈ 0.6° nakuplja se uglavnom u sporom završetku okreta (ω < 0.2 rad/s), dok kotači
  puze.

Odluka: zadano ostaje bez rampe (TURN_ALPHA_MAX_RAD_S2 = None) i B_eff = 0.3392 m. Pogreška okreta od
≈ 0.6° (σ ≈ 0.15°, CW malo lošiji) je karakteristika odometrije koju u radu ispravlja fuzija.
