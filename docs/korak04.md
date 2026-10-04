# Korak 4 – Diferencijalna odometrija

Iz prirasta impulsa lijevog i desnog enkodera u svakom uzorku (100 Hz):

ds_L = r·q_c·ΔN_L,  ds_R = r·q_c·ΔN_R,  ds = (ds_R + ds_L)/2,  Δθ = (ds_R − ds_L)/B_eff

Integracija sredinom intervala (midpoint):

θ_m = θ + Δθ/2;  x ← x + ds·cos θ_m;  y ← y + ds·sin θ_m;  θ ← wrap(θ + Δθ)

Točna je za pravac i rotaciju na mjestu; za luk je pogreška po koraku reda ds·Δθ², što je kod
koraka od 10 ms zanemarivo (četvrtina kružnice R = 1 m: odstupanje ispod 2 mm, na razini
kvantizacije enkodera).

Uz pozu se vode: predznačeni prijeđeni put (FSM pozicioniranje), ukupni put i neomotani kut
(relativni zavoji, +180° i −180° ostaju različiti). GT nikad nije ulaz.

Odometrija je lokalizacija konfiguracije A i ulaz predikcije EKF-a u konfiguracijama B–E.
