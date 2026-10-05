# Korak 8 – Vođenje po hodniku (PROX-NAV v2)

## 8a – Mjerni model hodnika

Četiri bočna senzora (L, FL, FR, R) mjere udaljenosti do zidova hodnika. Iz njih se tvore dvije
kombinacije:

    s = (d_R − d_L)/2        razlika bočnih senzora
    e =  d_FL − d_FR         razlika dijagonalnih senzora

Za zrake i male kutove (linearizacija) vrijedi:

    s ≈ y + x_s·α                     x_s = uzdužni odmak bočnih senzora od osi kotača
    e ≈ −(2/sin φ)·y − c·α            φ = kut dijagonalnih senzora prema smjeru vožnje

Dvije jednadžbe s dvije nepoznanice daju lateralni odmak y i kut α prema osi hodnika:

    [y, α]ᵀ = M⁻¹·([s, e]ᵀ − [s₀, e₀]ᵀ)

Senzori dakle opažaju i položaj i orijentaciju robota u hodniku. To je temelj vođenja i kasnije
mjernog modela PROX-LOC (konfiguracija B).

Kalibracija se radi live, s GT-om kao referencom (kao kalibracijski postav u laboratoriju; GT nije
ulaz regulatora). U START hodniku T1 izvodi se pet pokretanja: okret δ ∈ {−6, −3, 0, +3, +6}° na
STARTU, zatim 0.45 m ravno bez korekcije. Okret pokriva α pri y ≈ 0, a ravna vožnja pokriva y pri
stalnom α. Uzorak je valjan ako sva četiri bočna senzora detektiraju zid i ako je os < 0.42 m od
STARTA (dalje dijagonale gledaju u otvor raskrižja ROOT). Koeficijenti se dobivaju metodom
najmanjih kvadrata.

Offline provjera skripte na sintetičkom hodniku (montaže senzora iz starog projekta): rezultat je
s = 1.004·y + 0.166·α i e = −2.466·y − 1.272·α, što odgovara starom modelu.

### Rezultat 8a (live, 5. 10. 2026; 2168 uzoraka, |y| ≤ 40 mm, |α| ≤ 5.5°)

    s = +1.0042·y + 0.1668·α + 0.00035    rms 0.021 mm
    e = −2.4648·y − 1.2633·α − 0.00453    rms 0.273 mm

Inverzni model:

    ŷ = 1.4735·s + 0.1946·e + 0.00036     pogreška rms 0.026 mm
    α̂ = −2.8750·s − 1.1712·e − 0.00429    pogreška rms 0.015°

Fizikalno tumačenje koeficijenata po senzoru:
- **Bočni senzori:** uzdužni odmak od osi kotača 0.167 m, bočno 0.180 m od osi robota.
- **Dijagonalni senzori:** pod ≈ 54° prema smjeru vožnje, 0.31 m ispred osi kotača.

**Nalaz – asimetrija montaže.** Robot na STARTU je točno na sredini (x = 0) i poravnat
(π/2 ± 10⁻⁷ rad), a FR ipak mjeri 4.5 mm više od FL (R 0.7 mm više od L). Desna dijagonala je
montirana ≈ 5 mm bliže osi robota i ≈ 0.4° drugačijeg kuta. Kalibrirani pomak (α₀ = −0.25°) to
uklanja. Simetrični (nekalibrirani) model bi robota trajno držao zakrenutog za 0.25°.

## 8b – Zakon vođenja (navigation/corridor_guidance.py)

**FULL** (sva četiri bočna senzora vide zidove hodnika, očitanja u valjanim prozorima):

    α_ref = sat(−atan(ŷ/L), ±6°),   L = 0.5 m        (look-ahead: povratak na sredinu na putu ≈ L)
    ω     = sat(−K·(α̂ − α_ref), ±0.6 rad/s),   K = 3 s⁻¹

**Pamćenje osi hodnika:** u FULL se os sprema u odometrijskom okviru smjera,
os ← os + 0.2·wrap(θ_odo − α̂ − os). Dok je robot u hodniku, os prati zidove, pa drift odometrije
ne smeta.

**HOLD** (otvor, raskrižje – zidovi nisu svi vidljivi): ω = sat(−K·wrap(θ_odo − os)). Robot drži
naučeni smjer hodnika.

**Okret:** pri zadanom okretu os se pomakne za kut okreta, pa robot nakon okreta odmah drži os
novog hodnika.

Vođenje je isto u svim konfiguracijama A–E. Oblikuje fizički put robota, a procjenu poze (karta,
prepoznavanje raskrižja) ne dira.

Offline testovi (svijet s montažama izvedenim iz live fita):
- Konvergencija iz y = 50 mm, α = 4° na 2 m: |y| < 3 mm, |α| < 0.3°.
- U kratkom hodniku (0.45 m) kut 5° pada ispod 1.5°.
- HOLD kroz otvor od 1.2 m drži os unutar 0.05°.
- Os se nakon okreta od +90° pomakne točno za okret (ostatak < 0.6°).

## 8c – Live provjera u START hodniku T1

Četiri pokretanja: G0, G+5, G−5 (poremećaj okretom u mjestu na STARTU, vođenje uključeno) i O+5
(isti poremećaj, vođenje isključeno – referenca). Zidovi su vidljivi samo prvih ≈ 0.55 m, dalje
robot ide u HOLD. Offline očekivanje na ROOT-u: G±5 → |y| ≈ 1.5 mm, α ≈ 0°; O+5 → y ≈ 117 mm.

Kriteriji:
- G0: |y| ≤ 3 mm, |α| ≤ 0.3°
- G±5: |y| ≤ 10 mm, |α| ≤ 0.5°
- nema kolizije
