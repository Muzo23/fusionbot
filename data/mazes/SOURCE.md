# Real micromouse mazes

Source: https://github.com/micromouseonline/mazefiles (commit 548dcdeb359bbd3aca997371f8542ec9644840ce,
downloaded 8. 10. 2026). Collection of micromouse contest and training mazes in text format
(posts `o`, walls `---` / `|`, start `S`, goal `G`). The repository notes that files may contain
errors and duplicates; the files are used unchanged.

| file | origin (folder in the collection) |
|---|---|
| minimaze.txt | training |
| maze-train-10x5-a.txt | training |
| training-10x5-test1.txt | training |
| practice-8x8-a.txt | training |
| training-8x8-test2.txt | training |
| uk2016f.txt | classic (UK contest 2016, final) |
| japan2017ef.txt | classic (All Japan 2017, expert final) |

Conversion to the FusionBot lattice: `src/mazes/micromouse.py`.
