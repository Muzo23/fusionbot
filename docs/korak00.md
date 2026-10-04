# Korak 0 – Struktura projekta i konfiguracija robota

Robot je diferencijalni mobilni robot projektiran u Autodesk Fusion 360 i pojednostavljen za MuJoCo.
Sve konstante robota nalaze se u `src/core/robot_config.py`.

Ključna razlika: geometrijski razmak kotača B = 0.292 m (CAD) i efektivni razmak B_eff = 0.3392 m,
identificiran rotacijom na mjestu i kalibracijom odometrije. B_eff obuhvaća kontakt, klizanje i
popustljivost i koristi se u odometriji i inverznoj kinematici.

Izvedene veličine: 1 impuls enkodera = 0.325 mm puta kotača; 1.0 m/s traži 18.876 rad/s po kotaču
(83.3 % granice komande 22.65 rad/s).

Inverzna kinematika:  ω_L = (v − B_eff·ω/2)/r,  ω_R = (v + B_eff·ω/2)/r
