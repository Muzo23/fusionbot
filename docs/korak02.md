# Korak 2 – Inkrementalni enkoder

Idealni inkrementalni enkoder, 1024 impulsa po okretaju kotača, očitanje 100 Hz.

1. odmotavanje kuta: Δq = wrap(q − q_prev) (vrijedi dok je |Δq| < π; pri 22.65 rad/s kotač se
   u 10 ms okrene 0.23 rad)
2. akumulacija: θ ← θ + Δq
3. kvantizacija: N = ⌊θ / q_c⌋, q_c = 2π/1024 rad (0.325 mm puta kotača)
4. izlaz: ΔN = N − N_prev, brzina ω_enc = ΔN·q_c / T

Broj impulsa je funkcija apsolutnog kuta, kao kod stvarnog enkodera (brid na svakoj fiksnoj
oznaci). Zato nema gubitka gibanja manjeg od jednog impulsa i broj je točan i pri promjeni smjera.
U odnosu na raniju implementaciju (ostatak + odsijecanje prema nuli) razlika je najviše 1 impuls,
i to samo oko promjene smjera.

Koristi se samo pozicija zgloba. Svaka brzina u regulaciji dolazi iz impulsa; pogreška pojedinog
uzorka brzine je najviše q_c/T = 0.61 rad/s, srednja vrijednost je točna.
