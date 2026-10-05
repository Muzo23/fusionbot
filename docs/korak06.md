# Korak 6 – Proximity percepcija

Pet senzora: LEFT, FRONT_LEFT (+35°), FRONT, FRONT_RIGHT (−35°), RIGHT. Bridge filtrira okolinu
tako da senzori ne vide samog robota.

FRONT: front_blocked = detektirano i udaljenost ≤ 0.057581 m (izvedeno iz zaustavnog puta pri
0.25 m/s, validirano end-to-end; odvojeno od niže fizičke sigurnosne granice).

Otvori, jedan automat stanja po strani:
WALL → CANDIDATE (dijagonalni senzor izgubi zid) → CONFIRMED (bočni senzor izgubi zid) → PASSED
(bočni senzor ponovno vidi zid). Iz CANDIDATE natrag u WALL ako dijagonala opet vidi zid dok ga bočni
još vidi (treperenje na kraju zida). CONFIRMED i PASSED su zaključani: kad dijagonala ponovno vidi
daleki zid bočnog hodnika, otvor se ne poništava. Reset radi FSM tek nakon izlaska iz raskrižja.

Provjera na stvarnim snimljenim podacima (CoppeliaSim, pomaci po 5 mm): CANDIDATE na −0.875 m,
CONFIRMED na −0.530 m, PASSED na +0.200 m od otvora, ponovna detekcija dijagonale na −0.215 m ne
poništava otvor; prvi front_blocked je točno onaj 100 Hz uzorak na kojem je validirani run kočio.

Live provjera na raskrižju ROOT u T1: CONFIRMED se očekuje na 1.422634 − 0.530606 = 0.892 m
odometrijskog puta od STARTA.
