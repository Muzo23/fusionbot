# Korak 3 – PI regulator brzine kotača

Jedan regulator po kotaču, 100 Hz, sinkrono s enkoderom:

ω_f = pomični prosjek zadnjih 5 uzoraka ω_enc;  e = ω_ref − ω_f;  u = K_p·e + K_i·∫e dt,  u ∈ [−1, 1]

K_p = 0.08, K_i = 0.40 (preuzeto, validirano). Pomični prosjek izglađuje kvantizaciju brzine
enkodera (±0.61 rad/s po uzorku) uz kašnjenje ~20 ms.

Anti-windup (uvjetna integracija): ako bi izlaz bio u zasićenju i pogreška ga gura dalje u
zasićenje, prirast integrala u tom koraku se odbacuje. Integral zato nikad ne premaši vrijednost
pune komande.

Offline odziv u zatvorenoj petlji (motor 500 Hz + enkoder + inercija kotača J = 0.004 kg m²):
skok na 14.16 rad/s (0.75 m/s): nadvišenje 0.2 %, smirivanje u pojas 2 % za 0.77 s,
stacionarna pogreška < 0.01 rad/s.

reset() briše integral i povijest filtra; FSM ga traži pri svakom početku ili kraju manevra.
