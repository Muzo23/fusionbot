# Korak 1 – Model DC motora (500 Hz, simulator)

Lanac jednog kotača u svakom koraku dinamike (2 ms):

1. komanda u ∈ [−1, 1] (iz PI regulatora, drži se 10 ms)
2. PWM mrtva zona: u_dz = 0 za |u| ≤ 0.06, inače sign(u)(|u| − 0.06)/0.94
3. kašnjenje prvog reda (τ = 0.05 s), egzaktna diskretizacija: u_eff ← u_eff + (1 − e^(−Δt/τ))(u_dz − u_eff)
4. pravac moment–brzina: τ_m = τ_stall (u_eff − ω/ω₀), ograničen na ±τ_stall
5. trenje: τ = τ_m − c·sign(ω) − b·ω, ograničeno na ±τ_stall

Stacionarna brzina bez opterećenja: ω_ss = (τ_stall·u_dz − c)/(τ_stall/ω₀ + b).
Za puni napon ω_ss ≈ 22.69 rad/s, što se poklapa s granicom komande 22.65 rad/s.

Prijenos momenta u MuJoCo: callback vraća ciljnu brzinu ±ω₀ i maksimalnu silu |τ|. Moment motora
pada na nulu prije nego kotač dosegne ω₀, pa je ograničenje sile uvijek aktivno i primijenjeni
moment jednak je τ (izvor momenta ostvaren preko brzinskog regulatora zgloba).

ω u modelu je stvarna brzina zgloba: to je ispravno za fizikalni model motora. Povratna veza
brzine za regulaciju dolazi isključivo iz enkodera (korak 2).

Python referenca (core/motor_model.py) i Lua skripta (sim/motor_script.py) dijele iste konstante;
provjereno: 500 slučajnih ulaza, razlika Lua − Python = 0.
